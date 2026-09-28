#!/usr/bin/env python3
# /// script
# requires-python = ">=3.10"
# dependencies = ["pyusb>=1.2.1"]
# ///
"""
Minimal DfuSe CLI for the GD32F350 flash bootloader on the Huion HS611
(USB 28e9:0189).

Why not dfu-util?
-----------------
This bootloader does not handle the standard USB SET_INTERFACE request, so
dfu-util dies with "Cannot set alternate interface: LIBUSB_ERROR_OTHER". The
WebUSB flasher works because it talks to the DFU class interface directly and
never issues SET_INTERFACE. This tool does the same via libusb/pyusb.

Protocol (mirrors the working flasher):
  - control transfers are class/interface: 0x21 OUT, 0xA1 IN
  - DNLOAD wValue = block number (0 = DfuSe command, 2 = data)
  - DfuSe SET_ADDRESS = 0x21 + LE32 addr; ERASE = 0x41 + LE32 addr
  - after each DNLOAD, poll GETSTATUS until dfuDNLOAD-IDLE, then ABORT -> dfuIDLE
  - flash: SET_ADDRESS, 1024-byte page erase, 1024-byte data blocks

Only writes at/above APP_BASE (0x08004000); the 16 KB bootloader is protected.
"""

from __future__ import annotations

import argparse
import struct
import sys
import time

import usb.core
import usb.util

VID = 0x28E9
PID = 0x0189
INTERFACE = 0

APP_BASE = 0x08004000
FLASH_BASE = 0x08000000
FLASH_BYTES = 0x10000        # GD32F350R8T6: 64 KB
PAGE_SIZE = 1024
CHUNK = 1024                 # <= DFU wTransferSize (0x800)

OPT_BASE = 0x1FFFF800        # option-byte page: never a legal write/erase target
OPT_END = OPT_BASE + 16

# DFU class bRequests
DFU_DNLOAD = 0x01
DFU_UPLOAD = 0x02
DFU_GETSTATUS = 0x03
DFU_CLRSTATUS = 0x04
DFU_ABORT = 0x06
REQ_OUT = 0x21               # class, interface, host->device
REQ_IN = 0xA1                # class, interface, device->host

# DfuSe command bytes
DFUSE_SET_ADDRESS = 0x21
DFUSE_ERASE = 0x41

# DFU states
ST_DFU_IDLE = 2
ST_DNLOAD_SYNC = 3
ST_DNBUSY = 4
ST_DNLOAD_IDLE = 5
ST_ERROR = 10

STATUS_NAMES = {
    0: "OK", 1: "errTARGET", 2: "errFILE", 3: "errWRITE", 4: "errERASE",
    5: "errCHECK_ERASED", 6: "errPROG", 7: "errVERIFY", 8: "errADDRESS",
    9: "errNOTDONE", 10: "errFIRMWARE", 11: "errVENDOR", 12: "errUSBR",
    13: "errPOR", 14: "errUNKNOWN", 15: "errSTALLEDPKT",
}
STATE_NAMES = {
    0: "appIDLE", 1: "appDETACH", 2: "dfuIDLE", 3: "dfuDNLOAD-SYNC",
    4: "dfuDNBUSY", 5: "dfuDNLOAD-IDLE", 6: "dfuMANIFEST-SYNC",
    7: "dfuMANIFEST", 8: "dfuMANIFEST-WAIT-RESET", 9: "dfuUPLOAD-IDLE",
    10: "dfuERROR",
}

TIMEOUT_MS = 5000
POLL_MAX_MS = 500


class DfuError(RuntimeError):
    pass


def log(msg: str) -> None:
    print(msg, flush=True)


class DfuSe:
    def __init__(self) -> None:
        dev = usb.core.find(idVendor=VID, idProduct=PID)
        if dev is None:
            raise DfuError(
                "DFU device 28e9:0189 not found. Hold the tablet button (PA1), "
                "plug USB, release, then retry."
            )
        self.dev = dev
        try:
            dev.set_configuration()
        except usb.core.USBError:
            pass
        # Claim the interface directly; never send SET_INTERFACE.
        try:
            usb.util.claim_interface(dev, INTERFACE)
        except usb.core.USBError:
            try:
                dev.detach_kernel_driver(INTERFACE)
            except usb.core.USBError:
                pass
            usb.util.claim_interface(dev, INTERFACE)
        self.assumed_idle = False

    def close(self) -> None:
        try:
            usb.util.release_interface(self.dev, INTERFACE)
        except Exception:
            pass
        usb.util.dispose_resources(self.dev)

    # --- low level ---------------------------------------------------------

    def _out(self, request: int, value: int, data=None) -> None:
        self.dev.ctrl_transfer(REQ_OUT, request, value, INTERFACE, data, TIMEOUT_MS)

    def _in(self, request: int, value: int, length: int) -> bytes:
        ret = self.dev.ctrl_transfer(REQ_IN, request, value, INTERFACE, length, TIMEOUT_MS)
        return bytes(ret)

    def get_status(self) -> tuple[int, int, int]:
        data = self._in(DFU_GETSTATUS, 0, 6)
        if len(data) < 6:
            raise DfuError(f"GETSTATUS returned {len(data)} byte(s); expected 6")
        status, poll, state = data[0], data[1] | (data[2] << 8) | (data[3] << 16), data[4]
        return status, max(poll, 5), state

    def _clear_status(self) -> None:
        try:
            self._out(DFU_CLRSTATUS, 0, None)
        except usb.core.USBError:
            # Do not claim the device is idle if the request failed.
            self.assumed_idle = False
            return
        self.assumed_idle = True

    def _abort(self) -> None:
        self._out(DFU_ABORT, 0, None)
        self.assumed_idle = True

    # --- state machine -----------------------------------------------------

    def _await_state(self, target: int, timeout_ms: int) -> bool:
        deadline = time.monotonic() + timeout_ms / 1000
        while True:
            status, poll, state = self.get_status()
            if state == target:
                return True
            if state == ST_ERROR:
                self._clear_status()
            if time.monotonic() >= deadline:
                return False
            time.sleep(min(max(poll, 5), POLL_MAX_MS) / 1000)

    def recover_to_idle(self) -> None:
        if self.assumed_idle:
            return
        for _ in range(3):
            try:
                status, _poll, state = self.get_status()
                if state == ST_DFU_IDLE:
                    self.assumed_idle = True
                    return
                if state == ST_ERROR or status != 0:
                    self._clear_status()
                else:
                    self._abort()
                if self._await_state(ST_DFU_IDLE, 500):
                    self.assumed_idle = True
                    return
            except usb.core.USBError:
                self.assumed_idle = False
            time.sleep(0.02)
        raise DfuError("could not return the device to dfuIDLE")

    def _abort_to_idle(self) -> None:
        self._abort()
        if self._await_state(ST_DFU_IDLE, 500):
            return
        self._clear_status()
        if not self._await_state(ST_DFU_IDLE, 500):
            raise DfuError("device did not return to dfuIDLE after ABORT")

    def _dnload(self, block: int, data=None) -> None:
        self._out(DFU_DNLOAD, block, data)
        self.assumed_idle = False

    def _upload(self, block: int, length: int) -> bytes:
        return self._in(DFU_UPLOAD, block, length)

    def _await_download(self, context: str) -> None:
        deadline = time.monotonic() + 10
        saw_busy = False
        while True:
            status, poll, state = self.get_status()
            if status != 0:
                raise DfuError(f"{context}: DFU {STATUS_NAMES.get(status, status)}")
            if state in (ST_DNBUSY, ST_DNLOAD_SYNC):
                saw_busy = True
            elif state == ST_DNLOAD_IDLE:
                return
            elif state == ST_DFU_IDLE and saw_busy:
                return
            if time.monotonic() >= deadline:
                raise DfuError(f"{context}: timed out")
            time.sleep(min(max(poll, 5), POLL_MAX_MS) / 1000)

    def _command(self, cmd_byte: int, address: int, context: str) -> None:
        self.recover_to_idle()
        self._dnload(0, bytes([cmd_byte]) + struct.pack("<I", address))
        self._await_download(context)
        self._abort_to_idle()

    # --- operations --------------------------------------------------------

    def set_address(self, address: int) -> None:
        self._command(DFUSE_SET_ADDRESS, address, f"SET_ADDRESS 0x{address:08X}")

    def erase_page(self, address: int) -> None:
        self._command(DFUSE_ERASE, address, f"ERASE 0x{address:08X}")

    def write_raw(self, address: int, data: bytes) -> None:
        self.set_address(address)
        self._dnload(2, data)
        self._await_download(f"WRITE 0x{address:08X}")
        self._abort_to_idle()

    def read_raw(self, address: int, length: int) -> bytes:
        self.set_address(address)
        chunk = self._upload(2, length)
        self._abort_to_idle()
        return chunk

    def reset(self) -> None:
        # A zero-length DNLOAD at the entry address triggers manifest + reset.
        try:
            self.set_address(APP_BASE)
            self._dnload(0, None)
            self.get_status()
        except usb.core.USBError:
            pass  # device resets during this, a failure here is expected


def parse_int(text: str) -> int:
    return int(text, 0)


def validate_target(start: int, length: int) -> None:
    """Reject destructive ranges below the app base, past flash, or on option bytes."""
    if start < 0:
        raise DfuError("address must be non-negative")
    if length <= 0:
        raise DfuError("length must be positive")
    if start < APP_BASE:
        raise DfuError(
            f"refusing 0x{start:08X}: below app base 0x{APP_BASE:08X} "
            "(the 16 KB DFU bootloader is protected)"
        )
    if start + length > FLASH_BASE + FLASH_BYTES:
        raise DfuError(
            f"range 0x{start:08X}+0x{length:X} exceeds flash end "
            f"0x{FLASH_BASE + FLASH_BYTES:08X}"
        )
    if start < OPT_END and start + length > OPT_BASE:
        raise DfuError(f"range overlaps the option bytes at 0x{OPT_BASE:08X}")


def cmd_info(dfu: DfuSe, _args) -> int:
    status, poll, state = dfu.get_status()
    log(f"Device: {VID:04x}:{PID:04x}  status={STATUS_NAMES.get(status, status)} "
        f"state={STATE_NAMES.get(state, state)} poll={poll}ms")
    return 0


def cmd_erase(dfu: DfuSe, args) -> int:
    start = args.address
    length = args.length
    if start % PAGE_SIZE:
        raise DfuError(f"address 0x{start:08X} is not aligned to the {PAGE_SIZE}-byte page size")
    validate_target(start, length)
    pages = list(range(start, start + length, PAGE_SIZE))
    for i, page in enumerate(pages, 1):
        log(f"[{i}/{len(pages)}] erase 0x{page:08X}")
        dfu.erase_page(page)
    return 0


def cmd_dump(dfu: DfuSe, args) -> int:
    address, length, out = args.address, args.length, args.output
    if address < FLASH_BASE or address + length > FLASH_BASE + FLASH_BYTES:
        raise DfuError("range is outside the 64 KB flash")
    buf = bytearray()
    while len(buf) < length:
        n = min(CHUNK, length - len(buf))
        addr = address + len(buf)
        log(f"read 0x{addr:08X} ({len(buf) + n}/{length})")
        chunk = dfu.read_raw(addr, n)
        if len(chunk) < n:
            raise DfuError(f"short read at 0x{addr:08X}: {len(chunk)} < {n}")
        buf += chunk[:n]
    with open(out, "wb") as fh:
        fh.write(buf)
    log(f"wrote {len(buf)} bytes to {out}")
    return 0


def cmd_write(dfu: DfuSe, args) -> int:
    address = args.address
    data = open(args.input, "rb").read()
    if not data:
        raise DfuError("input file is empty")
    validate_target(address, len(data))

    log(f"device {VID:04x}:{PID:04x}, {STATUS_NAMES.get(dfu.get_status()[0])}")
    if not args.no_erase:
        first = address - (address % PAGE_SIZE)
        last = address + len(data) - 1
        pages = list(range(first, last + 1, PAGE_SIZE))
        for i, page in enumerate(pages, 1):
            log(f"[erase {i}/{len(pages)}] 0x{page:08X}")
            dfu.erase_page(page)

    for offset in range(0, len(data), CHUNK):
        chunk = data[offset:offset + CHUNK]
        addr = address + offset
        log(f"[write {offset + len(chunk)}/{len(data)}] 0x{addr:08X} ({len(chunk)} B)")
        dfu.write_raw(addr, chunk)

    if args.verify:
        log("verifying...")
        for offset in range(0, len(data), CHUNK):
            chunk = data[offset:offset + CHUNK]
            addr = address + offset
            got = dfu.read_raw(addr, len(chunk))
            if got[:len(chunk)] != chunk:
                raise DfuError(f"verify failed at 0x{addr:08X}")
        log("verify OK")

    if not args.no_reset:
        log("resetting into application")
        dfu.reset()
    return 0


def cmd_reset(dfu: DfuSe, _args) -> int:
    log("resetting into application")
    dfu.reset()
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="DfuSe CLI for GD32F350 HS611 (28e9:0189)")
    sub = parser.add_subparsers(dest="cmd", required=True)

    sub.add_parser("info", help="show DFU status/state").set_defaults(func=cmd_info)

    p = sub.add_parser("erase", help="erase pages (>= 0x08004000)")
    p.add_argument("address", type=parse_int)
    p.add_argument("length", type=parse_int)
    p.set_defaults(func=cmd_erase)

    p = sub.add_parser("dump", help="read flash to a file")
    p.add_argument("address", type=parse_int)
    p.add_argument("length", type=parse_int)
    p.add_argument("output")
    p.set_defaults(func=cmd_dump)

    p = sub.add_parser("write", help="erase+program an app image (>= 0x08004000)")
    p.add_argument("address", type=parse_int)
    p.add_argument("input")
    p.add_argument("--no-erase", action="store_true")
    p.add_argument("--verify", action="store_true")
    p.add_argument("--no-reset", action="store_true")
    p.set_defaults(func=cmd_write)

    sub.add_parser("reset", help="leave DFU and run the application").set_defaults(func=cmd_reset)

    args = parser.parse_args()
    dfu = DfuSe()
    try:
        return args.func(dfu, args)
    finally:
        dfu.close()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (DfuError, usb.core.USBError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1)
