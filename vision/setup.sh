# One-time setup for the object detector. Run from the HTN repo root:  sh vision/setup.sh
# Creates vision/.venv with OpenCV and ONNX Runtime, and downloads the MobileSAM model (44 MB, Apache 2.0).
cd "$(dirname "$0")" || exit 1
[ -d .venv ] || python3 -m venv .venv || exit 1
.venv/bin/pip install -q -r requirements.txt || exit 1
mkdir -p models
if [ ! -f models/mobile_sam.encoder.onnx ]; then
  curl -L -o models/mobile_sam.zip "https://huggingface.co/vietanhdev/segment-anything-onnx-models/resolve/main/mobile_sam_20230629.zip" || exit 1
  .venv/bin/python -c "import zipfile; zipfile.ZipFile('models/mobile_sam.zip').extractall('models')" && command rm -f models/mobile_sam.zip
fi
echo "ready. Scan the arena with: vision/.venv/bin/python vision/scan.py"
