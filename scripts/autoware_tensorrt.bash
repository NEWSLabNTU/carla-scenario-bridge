# The TensorRT runtime the Autoware stacks run on: the version Autoware was compiled against.
#
# Sourced by the justfile, never run. Autoware 1.5.0's libautoware_tensorrt_common was built
# against TensorRT 10.8.0 and accepts a cached engine only when the plan's TensorRT version
# equals that compile-time one. The loader, though, takes libnvinfer.so.10 from whatever the
# host offers first -- on a host with CUDA 12.8's TensorRT 10.9 under /usr/local/cuda, that
# one -- so every engine is built by 10.9, rejected at the next start ("Plan was created
# with a different version of TensorRT! Current version: 10.8.0"), and rebuilt: CenterPoint
# then spends minutes building at every ego-stack start.
#
# The fix is a 10.8.0 runtime first on LD_LIBRARY_PATH for the Autoware stacks only, from
# NVIDIA's pip wheel (tensorrt-cu12-libs, which carries libnvinfer, its builder resource,
# the plugin library and the ONNX parser), extracted into a plain directory. Nothing is
# installed into the system or a Python environment. The directory is a host setting,
# AUTOWARE_TENSORRT_LIB; `just setup-tensorrt` fills it.
#
# Bump all three together when Autoware moves to a release built against another TensorRT.

CSB_TENSORRT_VERSION=10.8.0
CSB_TENSORRT_WHEEL=tensorrt_cu12_libs-10.8.0.43-py2.py3-none-manylinux_2_28_x86_64.whl
CSB_TENSORRT_WHEEL_SHA256=aed2af55168297ebbf0de666fb2cef6bbab0dd64c516f390d406a81f8cbdbffe
CSB_TENSORRT_LIBS=(
    libnvinfer.so.10
    "libnvinfer_builder_resource.so.$CSB_TENSORRT_VERSION"
    libnvinfer_plugin.so.10
    libnvonnxparser.so.10
)

# Download the wheel and extract the libraries into $1. Idempotent: a complete directory is
# left alone. The wheel (3.1 GB) is downloaded beside $1 and removed afterwards.
csb_tensorrt_setup() {
    local dir="$1"
    [ -n "$dir" ] || { echo "[setup-tensorrt] AUTOWARE_TENSORRT_LIB is empty: nothing to do" >&2; return 0; }
    if _csb_tensorrt_complete "$dir"; then
        echo "[setup-tensorrt] TensorRT $CSB_TENSORRT_VERSION already in $dir"
        return 0
    fi
    mkdir -p "$dir"
    local whl="$dir/../$CSB_TENSORRT_WHEEL"
    if ! echo "$CSB_TENSORRT_WHEEL_SHA256  $whl" | sha256sum -c --status 2>/dev/null; then
        echo "[setup-tensorrt] downloading $CSB_TENSORRT_WHEEL (3.1 GB) from pypi.nvidia.com"
        curl -fL -# --retry 3 -o "$whl.part" "https://pypi.nvidia.com/tensorrt-cu12-libs/$CSB_TENSORRT_WHEEL"
        mv "$whl.part" "$whl"
        echo "$CSB_TENSORRT_WHEEL_SHA256  $whl" | sha256sum -c --status \
            || { echo "[setup-tensorrt] checksum mismatch: $whl" >&2; return 1; }
    fi
    local f members=()
    for f in "${CSB_TENSORRT_LIBS[@]}"; do members+=("tensorrt_libs/$f"); done
    unzip -q -o -j "$whl" "${members[@]}" "${CSB_TENSORRT_WHEEL%%-py2*}.dist-info/LICENSE.txt" -d "$dir"
    rm -f "$whl"
    _csb_tensorrt_complete "$dir" || { echo "[setup-tensorrt] extraction incomplete in $dir" >&2; return 1; }
    echo "[setup-tensorrt] TensorRT $CSB_TENSORRT_VERSION in $dir"
}

_csb_tensorrt_complete() {
    local f
    for f in "${CSB_TENSORRT_LIBS[@]}"; do [ -s "$1/$f" ] || return 1; done
}

# Put the runtime first on LD_LIBRARY_PATH of the calling shell, after the setups are
# sourced (they rewrite LD_LIBRARY_PATH). $1: the directory; empty turns it off. A missing
# or incomplete directory warns and leaves the host's TensorRT in place.
csb_tensorrt_source() {
    local dir="$1"
    if [ -z "$dir" ]; then
        echo "[just] AUTOWARE_TENSORRT_LIB is empty: the host's TensorRT, engines rebuilt per start" >&2
        return 0
    fi
    if ! _csb_tensorrt_complete "$dir"; then
        echo "[just] WARNING: no TensorRT $CSB_TENSORRT_VERSION in $dir: Autoware's TensorRT nodes" >&2
        echo "[just]   (CenterPoint, any TensorRT model) rebuild their engines at every start" >&2
        echo "[just]   when the host's TensorRT is another version. Fill it: just setup-tensorrt" >&2
        return 0
    fi
    export LD_LIBRARY_PATH="$dir${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
}
