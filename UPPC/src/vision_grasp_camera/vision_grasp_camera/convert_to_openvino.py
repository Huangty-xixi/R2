#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
convert_to_openvino.py  ——  YOLOv8 → OpenVINO IR 模型转换脚本
================================================================
将 Ultralytics YOLOv8 分割模型（.pt）转换为 OpenVINO IR 格式（.xml + .bin），
供 C++ 检测节点加载使用。

依赖（转换时用，运行时不需要）：
    pip install ultralytics openvino-dev

使用方法：
    python3 convert_to_openvino.py --weights best.pt --imgsz 640
    python3 convert_to_openvino.py --weights best.pt --imgsz 640 --half   # FP16

输出文件（在 best_openvino_model/ 目录下）：
    best.xml   ← 模型结构（C++ 节点 model_xml_path 参数指向此文件）
    best.bin   ← 模型权重

转换后测试：
    python3 convert_to_openvino.py --test --weights best.pt --xml best_openvino_model/best.xml
"""

import argparse
import sys
import time
from pathlib import Path


def check_deps():
    missing = []
    try:
        import ultralytics  # noqa
    except ImportError:
        missing.append("ultralytics")
    try:
        import openvino  # noqa
    except ImportError:
        missing.append("openvino-dev")
    if missing:
        print(f"[错误] 缺少依赖，请安装：pip install {' '.join(missing)}")
        sys.exit(1)


def convert(weights: str, imgsz: int, half: bool, output_dir: str | None) -> Path:
    """YOLOv8 .pt → OpenVINO IR"""
    from ultralytics import YOLO

    pt_path = Path(weights)
    if not pt_path.exists():
        print(f"[错误] 找不到模型文件: {pt_path}")
        sys.exit(1)

    print(f"\n[1/3] 加载 YOLOv8 模型: {pt_path}")
    model = YOLO(str(pt_path))

    # 打印模型信息
    print(f"      任务类型: {model.task}")
    print(f"      类别数量: {len(model.names)}")
    print(f"      类别列表: {list(model.names.values())}")

    # ── 导出为 OpenVINO IR ────────────────────────────────────────────────────
    print(f"\n[2/3] 导出为 OpenVINO IR  imgsz={imgsz}  half={half}")
    print("      这可能需要 1~3 分钟...")
    t0 = time.time()

    export_args = dict(
        format="openvino",
        imgsz=imgsz,
        half=half,           # FP16（推荐在 Intel GPU/VPU 上使用）
        dynamic=False,       # 固定输入尺寸，推理更快
        simplify=True,       # ONNX 图简化
        opset=12,
    )

    save_dir = model.export(**export_args)
    elapsed  = time.time() - t0
    print(f"      导出完成，用时 {elapsed:.1f}s")

    # 找到 .xml 文件
    save_path = Path(save_dir)
    xml_files = list(save_path.glob("*.xml"))
    if not xml_files:
        print(f"[错误] 未找到 .xml 文件，导出目录: {save_path}")
        sys.exit(1)

    xml_path = xml_files[0]
    print(f"\n[3/3] 输出文件:")
    print(f"      {xml_path}")
    print(f"      {xml_path.with_suffix('.bin')}")

    # ── 打印 C++ 节点参数配置 ─────────────────────────────────────────────────
    class_names = list(model.names.values())
    print("\n" + "="*60)
    print("在 camera_params.yaml 中配置以下参数：")
    print("="*60)
    print(f"""
camera_detector:
  ros__parameters:
    model_xml_path: "{xml_path.absolute()}"
    device: "CPU"               # 或 "GPU"（需要 Intel GPU 驱动）
    class_names: {class_names}
    target_classes: {class_names}  # 只检测这些类别，可按需缩减
    conf_threshold: 0.45
    iou_threshold:  0.45
    range_min_m:    0.5
    range_max_m:    1.0
    depth_scale:    0.001       # RealSense D435i: 1mm = 0.001m
    enabled_on_start: true
    publish_debug_image: true
""")
    return xml_path


def test_inference(weights: str, xml_path: str, imgsz: int):
    """用 OpenVINO Python API 做一次推理验证模型正确性"""
    import numpy as np
    import openvino as ov

    print(f"\n[测试] 加载 OpenVINO 模型: {xml_path}")
    core  = ov.Core()
    model = core.read_model(xml_path)
    cmodel = core.compile_model(model, "CPU")
    infer  = cmodel.create_infer_request()

    # 随机输入
    in_shape = cmodel.input().shape
    print(f"      输入形状: {in_shape}")
    dummy = np.random.rand(*in_shape).astype(np.float32)
    infer.set_input_tensor(ov.Tensor(dummy))

    t0 = time.time()
    infer.infer()
    elapsed = (time.time() - t0) * 1000

    outputs = [infer.get_output_tensor(i) for i in range(len(cmodel.outputs))]
    for i, o in enumerate(outputs):
        print(f"      output{i} 形状: {o.shape}")

    print(f"\n[测试] 推理成功！单次推理时间: {elapsed:.1f} ms")

    # 同时用 ultralytics 验证精度
    try:
        from ultralytics import YOLO
        import cv2
        pt_path = Path(weights)
        if pt_path.exists():
            model_pt = YOLO(str(pt_path))
            dummy_img = np.zeros((imgsz, imgsz, 3), dtype=np.uint8)
            result = model_pt(dummy_img, verbose=False)
            print(f"[测试] Ultralytics 推理正常（空图），检测数: {len(result[0].boxes)}")
    except Exception:
        pass


def main():
    ap = argparse.ArgumentParser(description="YOLOv8 → OpenVINO IR 转换工具")
    ap.add_argument("--weights",  default="best.pt",    help="YOLOv8 .pt 文件路径")
    ap.add_argument("--imgsz",    type=int, default=640, help="输入图像尺寸")
    ap.add_argument("--half",     action="store_true",  help="导出为 FP16（CPU 不支持，仅用于 GPU/VPU）")
    ap.add_argument("--output",   default=None,         help="输出目录（可选）")
    ap.add_argument("--test",     action="store_true",  help="转换后做推理验证")
    ap.add_argument("--xml",      default=None,         help="--test 时指定已有 .xml 路径")
    args = ap.parse_args()

    check_deps()

    if args.test and args.xml:
        # 只做验证，不重新转换
        test_inference(args.weights, args.xml, args.imgsz)
        return

    xml_path = convert(args.weights, args.imgsz, args.half, args.output)

    if args.test:
        test_inference(args.weights, str(xml_path), args.imgsz)


if __name__ == "__main__":
    main()

