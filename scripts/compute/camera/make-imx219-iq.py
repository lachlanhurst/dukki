#!/usr/bin/env python3
"""Build an RK3576 (ISP39) rkaiq tuning file for the IMX219 from Radxa's RK3588 (ISP30) IMX219 tuning.

The engine on the RK3576 validates tuning files against the ISP39 schema and Radxa ships no IMX219 file
for it. This takes an ISP39 file for a similar sensor as the skeleton (the Samsung S5K4H5YB from the
engine's own package: same 3280x2464 array, same 1.12 um pitch) and transplants the blocks that are
properties of the IMX219 module rather than of the ISP: sensor gain and exposure register mapping, black
level per ISO, white balance light sources and colour temperature line, colour correction matrices and
lens shading tables. Everything else (exposure loop, gamma, noise reduction, sharpening) keeps the
skeleton's values.

The field mapping was derived from the one module Radxa tuned in both formats, the IMX415 Radxa Camera
4K, by diffing its ISP30 and ISP39 files. See docs/camera-setup.md.

  make-imx219-iq.py --isp30 imx219_rpi-camera-v2_default.json --template s5k4h5yb_...json \
                    --out imx219_rpi-camera-v2_default.json [--lsc 3280x2464|1920x1080] [--engine-ae]

--lsc picks which of the ISP30 file's shading tables to carry (it has both); the ISP39 grid is relative
to the ISP input, so choose the sensor mode the pipeline runs. --engine-ae leaves the engine's own auto
exposure enabled (what setup-camera-3a.sh ships); without it the engine only does colour and mediad owns
exposure (upstream setup-rkaiq.sh style).
"""
import argparse, copy, json, sys

DOOR = {"CALIB_AWB_DOOR_TYPE_INDOOR": "awb_doorType_indoor",
        "CALIB_AWB_DOOR_TYPE_OUTDOOR": "awb_doorType_outdoor",
        "CALIB_AWB_DOOR_TYPE_AMBIGUITY": "awb_doorType_ambiguity"}

def load(p):
    return json.load(open(p, encoding="utf-8", errors="ignore"))

def scene(d, key):
    return d["main_scene"][0]["sub_scene"][0][key]

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--isp30", required=True); ap.add_argument("--template", required=True)
    ap.add_argument("--out", required=True); ap.add_argument("--lsc", default="3280x2464")
    ap.add_argument("--engine-ae", action="store_true")
    a = ap.parse_args()
    src = load(a.isp30); tpl = load(a.template)
    s30 = scene(src, "scene_isp30"); out = copy.deepcopy(tpl); s39 = scene(out, "scene_isp39")
    notes = []

    # ---- sensor_calib: the IMX219's register mapping, in the ISP39 shape
    sc, t = src["sensor_calib"], out["sensor_calib"]
    t["resolution"] = sc["resolution"]
    t["Gain2Reg"]["GainMode"] = sc["Gain2Reg"]["GainMode"]; t["Gain2Reg"]["GainRange"] = sc["Gain2Reg"]["GainRange"]
    t["Gain2Reg"]["GainRange_len"] = sc["Gain2Reg"]["GainRange_len"]
    t["Time2Reg"] = sc["Time2Reg"]; t["CISGainSet"] = sc["CISGainSet"]
    lin = t["CISTimeSet"]["Linear"]; lin.update({k: v for k, v in sc["CISTimeSet"]["Linear"].items()})
    t["CISHdrSet"] = sc["CISHdrSet"]; t["CISDcgSet"] = sc["CISDcgSet"]; t["CISExpUpdate"] = sc["CISExpUpdate"]
    t["CISMinFps"] = sc["CISMinFps"]; t["CISFlip"] = sc["CISFlip"]
    # The vendor imx219 driver's two gain controls are one total-gain handler, and the engine only ever
    # writes V4L2_CID_ANALOGUE_GAIN (256..2816, 1x..11x), so describe the sensor as 11x of analogue gain
    # and no digital gain. The source file's 170x (11x analogue times 16x digital) let the exposure loop
    # demand gains the driver cannot take ("GAIN OUT OF RANGE") and spiral to a dark frame. Headroom for
    # dark scenes comes from the ISP's own digital gain instead (up to 4x, see the route below).
    t["Gain2Reg"]["GainRange"] = [1, 11, 256, 0, 1, 256, 2816]; t["Gain2Reg"]["GainRange_len"] = 7
    t["CISGainSet"]["CISAgainRange"] = {"Min": 1, "Max": 11}
    t["CISGainSet"]["CISDgainRange"] = {"Min": 1, "Max": 1}
    t["CISGainSet"]["CISIspDgainRange"] = {"Min": 1, "Max": 4}
    out["module_calib"] = src["module_calib"]
    iso_list = t["iso_list"]

    # ---- black level per ISO
    blc = s30["ablc_calib"]["BlcTuningPara"]["BLC_Data"]
    dyn = s39["blc"]["stAuto"]["dyn"]
    if len(dyn) != len(iso_list) or len(blc["ISO"]) != len(iso_list):
        notes.append(f"blc: template has {len(dyn)} ISO entries, source {len(blc['ISO'])}, iso_list {len(iso_list)}")
    for i, d in enumerate(dyn):
        j = min(i, len(blc["R_Channel"]) - 1)
        d["obcPreTnr"] = {"hw_blcC_obR_val": blc["R_Channel"][j], "hw_blcC_obGr_val": blc["Gr_Channel"][j],
                          "hw_blcC_obGb_val": blc["Gb_Channel"][j], "hw_blcC_obB_val": blc["B_Channel"][j]}
    s39["blc"]["stMan"]["dyn"]["obcPreTnr"] = copy.deepcopy(dyn[0]["obcPreTnr"])

    # ---- white balance
    wb30 = s30["wb_v21"]; auto30 = wb30["autoPara"]; ext30 = wb30["autoExtPara"]
    wb39 = s39["wb"]; stats = wb39["awbStats"]
    by_name = {l["name"]: l for l in stats["lightSources"]}
    new_ls = []
    for l in auto30["lightSources"]:
        base = copy.deepcopy(by_name.get(l["name"], stats["lightSources"][0]))
        base["name"] = l["name"]
        base["doorType_mode"] = DOOR[l["doorType"]]
        base["standard_wbGain"] = l["standardGainValue"]
        base["wpDct_uvSpace"]["regionVtx"] = [{"hw_awbT_vtxU_val": u, "hw_awbT_vtxV_val": v}
                                              for u, v in zip(l["uvRegion"]["u"], l["uvRegion"]["v"])]
        for k in ("normal", "big"):
            x1, x2, y1, y2 = l["xyRegion"][k]
            base["wpDct_xySpace"][k] = {"ltVtx": {"hw_awbT_vtxX_val": x1, "hw_awbT_vtxY_val": y1},
                                        "rbVtx": {"hw_awbT_vtxX_val": x2, "hw_awbT_vtxY_val": y2}}
        r = l["rtYuvRegion"]; lv = r["lineVector"]
        base["wpDct_rotYuvSpace"]["hw_awbT_u2WpDistTh_curve"] = {"idx": r["thcurve_u"], "val": r["thcure_th"]}
        base["wpDct_rotYuvSpace"]["lsVect"]["edp"] = [
            {"hw_awbT_edpY_val": lv[2], "hw_awbT_edpU_val": lv[0], "hw_awbT_edpV_val": lv[1]},
            {"hw_awbT_edpY_val": lv[5], "hw_awbT_edpU_val": lv[3], "hw_awbT_edpV_val": lv[4]}]
        base["lgtPrefer"]["preferWbGain"] = [{"luma_val": l["dayGainLvThSet"][0], "prf_wbgain": l["defaultDayGainLow"]},
                                             {"luma_val": l["dayGainLvThSet"][1], "prf_wbgain": l["defaultDayGainHigh"]}]
        base["lgtPrefer"]["preferWbGain_len"] = 2
        new_ls.append(base)
    stats["lightSources"] = new_ls; stats["lightSources_len"] = len(new_ls)
    stats["rgb2xy"]["hw_awbCfg_rgb2xy_coeff"] = auto30["rgb2TcsPara"]["pseudoLuminanceWeight"]
    stats["rgb2xy"]["hw_awbCfg_xyTransMatrix_coeff"] = auto30["rgb2TcsPara"]["rotationMat"][:6]
    stats["hw_awbCfg_rgb2RotYuv_coeff"] = auto30["rgb2RotationYuvMat"][:12]
    first = ext30["lightSourceForFirstFrame"]
    first_gain = next(l["standardGainValue"] for l in auto30["lightSources"] if l["name"] == first)
    wb39["awbGnCalcOth"]["fstFrm_wbgain"] = first_gain
    ct = wb39["awbGnCalcOth"]["ctCalc"]
    ct["lineRgBg"] = dict(zip("abc", ext30["lineRgBg"])); ct["lineRgProjCCT"] = dict(zip("abc", ext30["lineRgProjCCT"]))
    wb39["awbGnCalcStep"]["wbGnType1"]["refWbGain"][0]["ref_wbgain"] = ext30["defaultNightGain"]
    wb39["wbGainCtrl"]["manualPara"]["cfg"]["manual_wbgain"] = wb30["manualPara"]["cfg"]["mwbGain"]

    # ---- colour correction
    ccm30 = s30["ccm_calib"]; ccm39 = s39["ccm"]
    ccm39["calibdb"]["matrixAll"] = [{"sw_ccmC_illu_name": m["illumination"], "sw_ccmC_ccmSat_val": m["saturation"],
                                      "ccMatrix": {"hw_ccmC_matrix_coeff": m["ccMatrix"], "hw_ccmC_matrix_offset": m["ccOffsets"]}}
                                     for m in ccm30["TuningPara"]["matrixAll"]]
    ccm39["calibdb"]["sw_ccmC_matrixAll_len"] = len(ccm39["calibdb"]["matrixAll"])
    ccm39["tunning"]["stAuto"]["dyn"]["illuLink"] = [
        {"sw_ccmC_illu_name": c["name"], "sw_ccmC_wbGainR_val": c["awbGain"][0], "sw_ccmC_wbGainB_val": c["awbGain"][1],
         "gain2SatCurve": {"sw_ccmT_isoIdx_val": c["gain_sat_curve"]["gains"], "sw_ccmT_glbSat_val": c["gain_sat_curve"]["sat"]}}
        for c in ccm30["TuningPara"]["aCcmCof"]]
    ccm39["tunning"]["stAuto"]["dyn"]["sw_ccmT_illuLink_len"] = len(ccm30["TuningPara"]["aCcmCof"])
    ccm39["tunning"]["stAuto"]["sta"]["ccmCfg"]["hw_ccmCfg_rgb2y_coeff"] = ccm30["lumaCCM"]["rgb2y_para"]
    ccm39["tunning"]["stMan"]["sta"]["hw_ccmCfg_rgb2y_coeff"] = ccm30["lumaCCM"]["rgb2y_para"]

    # ---- lens shading
    lsc30 = s30["lsc_v2"]; lsc39 = s39["lsc"]
    tables = [t for t in lsc30["tbl"]["tableAll"] if t["resolution"] == a.lsc]
    if not tables:
        sys.exit(f"no ISP30 shading tables for resolution {a.lsc}; have {sorted(set(t['resolution'] for t in lsc30['tbl']['tableAll']))}")
    lsc39["calibdb"]["tableAll"] = [{"sw_lscC_illu_name": t["illumination"], "sw_lscC_vignetting_val": t["vignetting"],
                                     "meshGain": {"hw_lscC_gainR_val": t["lsc_samples_red"]["uCoeff"],
                                                  "hw_lscC_gainGr_val": t["lsc_samples_greenR"]["uCoeff"],
                                                  "hw_lscC_gainB_val": t["lsc_samples_blue"]["uCoeff"],
                                                  "hw_lscC_gainGb_val": t["lsc_samples_greenB"]["uCoeff"]}}
                                    for t in tables]
    lsc39["calibdb"]["tableAll_len"] = len(tables)
    lsc39["tunning"]["stAuto"]["dyn"]["illuLink"] = [
        {"sw_lscC_illu_name": i["name"], "sw_lscC_wbGainR_val": i["wbGain"][0], "sw_lscC_wbGainB_val": i["wbGain"][1],
         "gain2VigCurve": {"sw_lscT_isoIdx_val": i["gains"], "sw_lscT_vignetting_val": i["vig"]}}
        for i in lsc30["alscCoef"]["illAll"]]
    lsc39["tunning"]["stAuto"]["dyn"]["sw_lscT_illuLink_len"] = len(lsc30["alscCoef"]["illAll"])

    # ---- exposure ownership
    s39["ae_calib"]["commCtrl"]["sw_aeT_algo_en"] = 1 if a.engine_ae else 0
    # Exposure route: time first up to 30 ms (motion blur on a walking robot), then sensor analogue gain
    # to its 11x, then ISP digital gain to 4x. Same shape as the template's route with the gain ceiling
    # the driver actually has.
    route = s39["ae_calib"]["linAeCtrl"]["route"]
    route["sw_aeT_route_len"] = 6
    route["sw_aeT_time_dot"] = [0, 0.03, 0.03, 0.03, 0.03, 0.03]
    route["sw_aeT_gain_dot"] = [1, 1, 4, 11, 11, 11]
    route["sw_aeT_ispDGain_dot"] = [1, 1, 1, 1, 2, 4]

    json.dump(out, open(a.out, "w"), indent=1)
    print(f"wrote {a.out}")
    print(f"  light sources {[l['name'] for l in new_ls]}, ccm matrices {len(ccm39['calibdb']['matrixAll'])}, "
          f"lsc tables {len(tables)} at {a.lsc}, blc ISO entries {len(dyn)}, engine AE {'on' if a.engine_ae else 'off'}")
    for n in notes: print("  note:", n)

if __name__ == "__main__":
    main()
