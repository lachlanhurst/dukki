#!/usr/bin/env python3
"""Estimate the JLCPCB assembly parts cost for a BOM from JLC's live part prices.

Reads a JLC BOM (Comment, Designator, Footprint, LCSC Part #) and, for each line, asks JLC's parts
library for the price tiers, minimum order and attrition, then prices the quantity JLC would buy
for the given number of assembled boards. Fees are JLC's published Economic PCBA ones
(https://jlcpcb.com/help/article/40-pcb-assembly-price): setup, stencil, per joint, and US$3 per
Extended part type (Preferred parts exempt). Prints US$ and A$ at the given exchange rate.

Usage: python3 scripts/kicad/jlc_cost.py hardware/bridge/fab/bridge-bom.csv --boards 5 [--aud 1.44]
"""
import argparse, csv, json, urllib.request

ap = argparse.ArgumentParser()
ap.add_argument("bom")
ap.add_argument("--boards", type=int, default=5)
ap.add_argument("--aud", type=float, default=1.44, help="A$ per US$ (1.4434 on 01/10/2026)")
ap.add_argument("--joints-per-board", type=int, default=0, help="override the solder joint count")
args = ap.parse_args()

SETUP, STENCIL, PER_JOINT, EXTENDED_FEE = 8.00, 1.50, 0.0016, 3.00

def lookup(code):
    req = urllib.request.Request(
        "https://jlcpcb.com/api/overseas-pcb-order/v1/shoppingCart/smtGood/selectSmtComponentList",
        data=json.dumps({"keyword": code, "currentPage": 1, "pageSize": 5}).encode(),
        headers={"Content-Type": "application/json", "User-Agent": "Mozilla/5.0"})
    rows = json.load(urllib.request.urlopen(req, timeout=30))["data"]["componentPageInfo"]["list"] or []
    for r in rows:
        if r["componentCode"] == code:
            return r
    raise SystemExit(f"{code} not found in JLC's library")

def unit_price(tiers, qty):
    price = tiers[0]["productPrice"]
    for t in tiers:
        if qty >= t["startNumber"]:
            price = t["productPrice"]
    return price

PADS = {"SOT-23-6": 6, "TO-252-2": 3}       # everything else in this BOM has two pads
lines, parts_total, ext_types, joints = [], 0.0, 0, 0
with open(args.bom) as f:
    for row in csv.DictReader(f):
        refs = row["Designator"].split(",")
        code = row["LCSC Part #"]
        r = lookup(code)
        need = len(refs) * args.boards
        buy = max(need + (r.get("lossNumber") or 0), r.get("leastPatchNumber") or 0, r.get("minPurchaseNum") or 1)
        cost = buy * unit_price(r["componentPrices"], buy)
        lib = "Basic" if r["componentLibraryType"] == "base" else ("Preferred" if r.get("preferredComponentFlag") else "Extended")
        if lib == "Extended":
            ext_types += 1
        joints += len(refs) * PADS.get(row["Footprint"], 2)
        parts_total += cost
        lines.append((row["Comment"], code, lib, len(refs), buy, cost, r["stockCount"]))

joints = args.joints_per_board or joints
fees = SETUP + STENCIL + EXTENDED_FEE * ext_types + PER_JOINT * joints * args.boards
print(f"{'Part':22} {'LCSC':11} {'Library':9} {'/board':>6} {'buy':>5} {'US$':>7} {'stock':>8}")
for name, code, lib, per, buy, cost, stock in lines:
    print(f"{name[:22]:22} {code:11} {lib:9} {per:>6} {buy:>5} {cost:>7.2f} {stock:>8}")
print(f"\nboards assembled: {args.boards}   joints per board: {joints}   Extended part types: {ext_types}")
print(f"parts      US$ {parts_total:7.2f}   A$ {parts_total * args.aud:7.2f}")
print(f"fees       US$ {fees:7.2f}   A$ {fees * args.aud:7.2f}   (setup {SETUP}, stencil {STENCIL}, "
      f"extended {EXTENDED_FEE * ext_types:.2f}, joints {PER_JOINT * joints * args.boards:.2f})")
print(f"assembly   US$ {parts_total + fees:7.2f}   A$ {(parts_total + fees) * args.aud:7.2f}   "
      f"(bare boards and shipping not included; rate A${args.aud} per US$)")
