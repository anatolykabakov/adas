#!/bin/bash

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

CPP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$CPP_DIR/../../../.." && pwd)"
BUILDOZER_PLATFORM="${BUILDOZER_PLATFORM:-}"

BUILD_DIR="build"
ABI="arm64-v8a"
PLATFORM="android-26"
BUILD_TYPE="Release"
CLEAN_BUILD=false
VERBOSE=false
BUILD_TARGET="android"
BUILD_TESTS=false
JNI_LIBS_DIR=""
CONAN_RUNTIME_DEPLOY_DIR=""

show_help() {
    echo "Usage: $0 [OPTIONS]"
    echo
    echo "OPTIONS:"
    echo "  -t, --target TARGET     Build target (android|linux) [default: android]"
    echo "  -a, --abi ABI           Architecture (arm64-v8a|armeabi-v7a|x86|x86_64) [default: arm64-v8a]"
    echo "  -p, --platform PLAT     Android platform (android-21|android-23|etc) [default: android-26]"
    echo "  -b, --type TYPE         Build type (Debug|Release) [default: Release]"
    echo "  -c, --clean             Clean before build"
    echo "  -v, --verbose           Verbose output"
    echo "  --test                  Build and run tests (Linux only)"
    echo "  -h, --help              Show this help"
    echo
    echo "Env: ANDROID_NDK_ROOT / ANDROID_NDK_HOME, BUILDOZER_PLATFORM"
    echo
    echo "EXAMPLES:"
    echo "  $0                      # Android arm64-v8a build"
    echo "  $0 -t linux             # Linux build"
    echo "  $0 -t android -c -v     # Clean verbose Android build"
    echo "  $0 -t linux --test      # Linux build and run tests"
}

parse_arguments() {
    while [[ $# -gt 0 ]]; do
        case $1 in
            -t|--target)
                BUILD_TARGET="$2"
                shift 2
                ;;
            -a|--abi)
                ABI="$2"
                shift 2
                ;;
            -p|--platform)
                PLATFORM="$2"
                shift 2
                ;;
            -b|--type)
                BUILD_TYPE="$2"
                shift 2
                ;;
            -c|--clean)
                CLEAN_BUILD=true
                shift
                ;;
            -v|--verbose)
                VERBOSE=true
                shift
                ;;
            --test)
                BUILD_TESTS=true
                shift
                ;;
            -h|--help)
                show_help
                exit 0
                ;;
            *)
                print_error "Unknown option: $1"
                show_help
                exit 1
                ;;
        esac
    done
}

resolve_ndk_path() {
    if [ -n "${ANDROID_NDK_ROOT:-}" ] && [ -d "$ANDROID_NDK_ROOT" ]; then
        NDK_PATH="$ANDROID_NDK_ROOT"
        return 0
    fi
    if [ -n "${ANDROID_NDK_HOME:-}" ] && [ -d "$ANDROID_NDK_HOME" ]; then
        NDK_PATH="$ANDROID_NDK_HOME"
        return 0
    fi
    if [ -n "${BUILDOZER_PLATFORM:-}" ] && [ -d "$BUILDOZER_PLATFORM/android-ndk-r28c" ]; then
        NDK_PATH="$BUILDOZER_PLATFORM/android-ndk-r28c"
        return 0
    fi
    if [ -f "$PROJECT_DIR/local.properties" ]; then
        local ndk
        ndk=$(grep -E '^ndk\.dir=' "$PROJECT_DIR/local.properties" | head -1 | cut -d= -f2- | tr -d '\r')
        if [ -n "$ndk" ] && [ -d "$ndk" ]; then
            NDK_PATH="$ndk"
            return 0
        fi
    fi
    return 1
}

check_dependencies() {
    print_status "Checking dependencies for $BUILD_TARGET..."

    if [ "$BUILD_TESTS" = true ] && [ "$BUILD_TARGET" != "linux" ]; then
        print_error "Tests require Linux target (--test only works with -t linux)"
        exit 1
    fi

    if ! command -v cmake &> /dev/null; then
        print_error "CMake not found"
        exit 1
    fi

    if ! command -v conan &> /dev/null; then
        print_error "Conan not found (pip install conan)"
        exit 1
    fi

    if [ ! -d "$CPP_DIR" ] || [ ! -f "$CPP_DIR/CMakeLists.txt" ]; then
        print_error "C++ directory not found: $CPP_DIR"
        exit 1
    fi

    if [ "$BUILD_TARGET" = "android" ]; then
        if ! resolve_ndk_path; then
            print_error "Android NDK not found (ANDROID_NDK_ROOT / buildozer / local.properties)"
            exit 1
        fi
        if [ ! -f "$NDK_PATH/build/cmake/android.toolchain.cmake" ]; then
            print_error "NDK toolchain not found: $NDK_PATH"
            exit 1
        fi
        export ANDROID_NDK_ROOT="$NDK_PATH"
        export ANDROID_NDK_HOME="$NDK_PATH"
    elif [ "$BUILD_TARGET" = "linux" ]; then
        if [ "$BUILD_TESTS" = true ]; then
            if [ ! -f "$CPP_DIR/tests/CMakeLists.txt" ]; then
                print_error "Test CMakeLists.txt not found: $CPP_DIR/tests/CMakeLists.txt"
                exit 1
            fi
        fi
    else
        print_error "Unknown build target: $BUILD_TARGET"
        print_error "Supported targets: android, linux"
        exit 1
    fi

    print_success "All dependencies found"
}

clean_build() {
    if [ "$CLEAN_BUILD" = true ]; then
        print_status "Cleaning previous build..."
        cd "$CPP_DIR"
        if [ -d "$BUILD_DIR" ]; then
            rm -rf "$BUILD_DIR"
            print_success "Build directory cleaned"
        fi
    fi
}

android_ndk_triple() {
    case "$1" in
        arm64-v8a) echo "aarch64-linux-android" ;;
        armeabi-v7a) echo "arm-linux-androideabi" ;;
        x86) echo "i686-linux-android" ;;
        x86_64) echo "x86_64-linux-android" ;;
        *) return 1 ;;
    esac
}

conan_install() {
    local -a args=(-of "$CPP_DIR/$BUILD_DIR" --build=missing -s "build_type=$BUILD_TYPE")

    if [ "$BUILD_TARGET" = "android" ]; then
        local clang_ver=14
        if [ -x "$NDK_PATH/toolchains/llvm/prebuilt/linux-x86_64/bin/clang" ]; then
            clang_ver=$("$NDK_PATH/toolchains/llvm/prebuilt/linux-x86_64/bin/clang" --version | head -1 | sed -E 's/.*clang version ([0-9]+).*/\1/')
        fi
        local ndk_policy_tc="$CPP_DIR/cmake/ndk_cmp0057.cmake"
        CONAN_RUNTIME_DEPLOY_DIR="$CPP_DIR/$BUILD_DIR/runtime_deploy"
        rm -rf "$CONAN_RUNTIME_DEPLOY_DIR"
        args+=(
            -s:h os=Android -s:h os.api_level=26 -s:h arch=armv8
            -s:h compiler=clang -s:h "compiler.version=$clang_ver" -s:h compiler.libcxx=c++_shared
            -s:h compiler.cppstd=17
            -c:h "tools.android:ndk_path=$NDK_PATH"
            -c:h "tools.cmake.cmaketoolchain:user_toolchain=[\"$ndk_policy_tc\"]"
            -c:h 'tools.cmake:configure_args=["-DCMAKE_POLICY_DEFAULT_CMP0057=NEW"]'
            --deployer=runtime_deploy
            --deployer-folder="$CONAN_RUNTIME_DEPLOY_DIR"
        )
    else
        args+=(-s:h os=Linux -s:h arch=x86_64 -o "&:python_bindings=True")
        if [ "$BUILD_TESTS" = true ]; then
            args+=(-o "&:tests=True")
        fi
    fi

    print_status "Conan install ($BUILD_TARGET)..."
    (cd "$CPP_DIR" && conan install . "${args[@]}")
}

conan_toolchain() {
    if [ -f "$CPP_DIR/$BUILD_DIR/conan_toolchain.cmake" ]; then
        echo "$CPP_DIR/$BUILD_DIR/conan_toolchain.cmake"
    elif [ -f "$CPP_DIR/$BUILD_DIR/generators/conan_toolchain.cmake" ]; then
        echo "$CPP_DIR/$BUILD_DIR/generators/conan_toolchain.cmake"
    else
        print_error "conan_toolchain.cmake not found in $CPP_DIR/$BUILD_DIR"
        exit 1
    fi
}

conan_activate_build_env() {
    local env_sh="$CPP_DIR/$BUILD_DIR/conanbuild.sh"
    if [ ! -f "$env_sh" ]; then
        env_sh="$CPP_DIR/$BUILD_DIR/generators/conanbuild.sh"
    fi
    if [ -f "$env_sh" ]; then
        # shellcheck disable=SC1090
        source "$env_sh"
        print_status "Conan build env: $(command -v protoc || echo 'protoc not on PATH')"
        if command -v protoc >/dev/null 2>&1; then
            print_status "  protoc: $(protoc --version 2>/dev/null || true)"
        fi
    else
        print_warning "conanbuild.sh not found — system protoc may not match Conan protobuf"
    fi
}

build_linux() {
    print_status "Building for Linux..."

    cd "$CPP_DIR"
    mkdir -p "$BUILD_DIR"

    conan_install
    local toolchain
    toolchain=$(conan_toolchain)
    conan_activate_build_env

    print_status "CMake configure (Linux)..."
    print_status "  - Build Type: $BUILD_TYPE"
    print_status "  - Toolchain: $toolchain"

    local -a cmake_args=(
        -DCMAKE_TOOLCHAIN_FILE="$toolchain"
        -DCMAKE_BUILD_TYPE="$BUILD_TYPE"
        -DBUILD_FOR_ANDROID=OFF
        -DBUILD_PYTHON_BINDINGS=ON
        -DCMAKE_PREFIX_PATH="$CPP_DIR/$BUILD_DIR"
        -UProtobuf_PROTOC_EXECUTABLE
        -UPROTOC_PROGRAM
    )
    if command -v protoc >/dev/null 2>&1; then
        cmake_args+=(-DProtobuf_PROTOC_EXECUTABLE="$(command -v protoc)")
    fi
    if [ "$BUILD_TESTS" = true ]; then
        cmake_args+=(-DBUILD_TESTING=ON)
        print_status "  - Build Testing: ON"
    else
        cmake_args+=(-DBUILD_TESTING=OFF)
        print_status "  - Build Testing: OFF"
    fi

    cmake "${cmake_args[@]}" -B "$BUILD_DIR" -S .

    print_status "Compiling C++ (Linux)..."
    if [ "$VERBOSE" = true ]; then
        cmake --build "$BUILD_DIR" --verbose
    else
        cmake --build "$BUILD_DIR"
    fi

    if [ "$BUILD_TESTS" = true ]; then
        echo
        print_status "Running tests..."
        TEST_EXECUTABLE="$BUILD_DIR/tests/adas_tests"
        if [ -f "$TEST_EXECUTABLE" ]; then
            cd "$BUILD_DIR"
            if ./tests/adas_tests; then
                print_success "All tests passed"
            else
                exit_code=$?
                print_error "Some tests failed (exit code: $exit_code)"
                exit 1
            fi
            cd "$CPP_DIR"
        else
            print_error "Test executable not found: $TEST_EXECUTABLE"
            exit 1
        fi
    fi
}

build_android() {
    print_status "Building for Android..."

    cd "$CPP_DIR"
    mkdir -p "$BUILD_DIR"

    print_status "CMake configure..."
    print_status "  - ABI: $ABI"
    print_status "  - Platform: $PLATFORM"
    print_status "  - Build Type: $BUILD_TYPE"
    print_status "  - NDK: $NDK_PATH"

    export ANDROID_NDK_HOME="$NDK_PATH"
    export ANDROID_ABI="$ABI"
    export ANDROID_PLATFORM="$PLATFORM"

    conan_install
    local toolchain
    toolchain=$(conan_toolchain)
    conan_activate_build_env

    local -a cmake_args=(
        -DCMAKE_TOOLCHAIN_FILE="$toolchain"
        -DCMAKE_BUILD_TYPE="$BUILD_TYPE"
        -DANDROID_ABI="$ABI"
        -DANDROID_PLATFORM="$PLATFORM"
        -DBUILD_FOR_ANDROID=ON
        -DCMAKE_PREFIX_PATH="$CPP_DIR/$BUILD_DIR"
        -UProtobuf_PROTOC_EXECUTABLE
        -UPROTOC_PROGRAM
    )
    if command -v protoc >/dev/null 2>&1; then
        cmake_args+=(-DProtobuf_PROTOC_EXECUTABLE="$(command -v protoc)")
    fi

    cmake "${cmake_args[@]}" -B "$CPP_DIR/$BUILD_DIR" -S "$CPP_DIR"

    print_status "Compiling C++..."
    if [ "$VERBOSE" = true ]; then
        cmake --build "$CPP_DIR/$BUILD_DIR" --verbose
    else
        cmake --build "$CPP_DIR/$BUILD_DIR"
    fi

    LIBRARY_PATH="$CPP_DIR/$BUILD_DIR/libadas_app_android.so"
    if [ ! -f "$LIBRARY_PATH" ]; then
        LIBRARY_PATH=$(find "$CPP_DIR/$BUILD_DIR" -name 'libadas_app_android.so' -type f | head -1 || true)
    fi

    if [ -n "$LIBRARY_PATH" ] && [ -f "$LIBRARY_PATH" ]; then
        BUILT_LIBRARY="$LIBRARY_PATH"
        print_success "C++ library built: $LIBRARY_PATH"
        echo
        print_status "Library info:"
        ls -la "$LIBRARY_PATH"
        echo
        print_status "Size: $(du -h "$LIBRARY_PATH" | cut -f1)"
        print_status "Architecture: $ABI"
        print_status "Platform: $PLATFORM"

        if command -v readelf &> /dev/null; then
            echo
            print_status "Library dependencies:"
            readelf -d "$LIBRARY_PATH" | grep NEEDED || echo "  None"
        fi
    else
        print_error "C++ library was not built"
        return 1
    fi
}

build_cpp_library() {
    if [ "$BUILD_TARGET" = "android" ]; then
        build_android
    elif [ "$BUILD_TARGET" = "linux" ]; then
        build_linux
    else
        print_error "Unknown build target: $BUILD_TARGET"
        exit 1
    fi
}

copy_to_jnilibs() {
    if [ "$BUILD_TARGET" != "android" ]; then
        print_status "Skipping jniLibs copy (not an Android build)"
        return 0
    fi

    JNI_LIBS_DIR="$PROJECT_DIR/app/libs/$ABI"
    CONAN_RUNTIME_DEPLOY_DIR="${CONAN_RUNTIME_DEPLOY_DIR:-$CPP_DIR/$BUILD_DIR/runtime_deploy}"

    print_status "Copying libraries to jniLibs ($ABI)..."
    rm -rf "$JNI_LIBS_DIR"
    mkdir -p "$JNI_LIBS_DIR"

    local copied=0
    if [ -d "$CONAN_RUNTIME_DEPLOY_DIR" ]; then
        local f base
        for f in "$CONAN_RUNTIME_DEPLOY_DIR"/lib*.so; do
            [ -e "$f" ] || continue
            base=$(basename "$f")
            case "$base" in
                *.so.*) continue ;;
                libprotoc.so) continue ;;
            esac
            cp -L "$f" "$JNI_LIBS_DIR/$base"
            copied=$((copied + 1))
        done
        print_status "  Conan runtime_deploy: $copied shared lib(s) -> $JNI_LIBS_DIR"
    else
        print_warning "runtime_deploy not found: $CONAN_RUNTIME_DEPLOY_DIR"
    fi

    local triple
    if ! triple=$(android_ndk_triple "$ABI"); then
        print_error "Unknown ABI for libc++_shared: $ABI"
        exit 1
    fi
    local cxx_shared="$NDK_PATH/toolchains/llvm/prebuilt/linux-x86_64/sysroot/usr/lib/$triple/libc++_shared.so"
    if [ ! -f "$cxx_shared" ]; then
        print_error "libc++_shared.so not found: $cxx_shared"
        exit 1
    fi
    cp -f "$cxx_shared" "$JNI_LIBS_DIR/libc++_shared.so"

    if [ -z "${BUILT_LIBRARY:-}" ] || [ ! -f "$BUILT_LIBRARY" ]; then
        print_error "Built library not found (BUILT_LIBRARY)"
        exit 1
    fi
    cp -f "$BUILT_LIBRARY" "$JNI_LIBS_DIR/libadas_app_android.so"

    # Раннер thneed — отдельная библиотека и НЕ обязательная. Она грузится только когда
    # vision.model_runner = "thneed", и Java ловит отказ загрузки. Поэтому её отсутствие не ошибка:
    # предупреждаем и идём дальше, иначе сборка ломалась бы у всех, кому этот путь не нужен.
    local thneed_lib
    thneed_lib=$(find "$CPP_DIR/$BUILD_DIR" -name "libthneedrunner.so" -print -quit 2>/dev/null)
    if [ -n "$thneed_lib" ] && [ -f "$thneed_lib" ]; then
        cp -f "$thneed_lib" "$JNI_LIBS_DIR/libthneedrunner.so"
        print_status "  thneed runner -> $JNI_LIBS_DIR/libthneedrunner.so"
    else
        print_warning "libthneedrunner.so not built — the thneed model runner will be unavailable"
    fi

    print_success "jniLibs ready -> $JNI_LIBS_DIR"
    ls -la "$JNI_LIBS_DIR"
}

show_build_info() {
    echo
    print_status "C++ build info:"
    echo "  - Target: $BUILD_TARGET"
    if [ "$BUILD_TARGET" = "android" ]; then
        echo "  - ABI: $ABI"
        echo "  - Platform: $PLATFORM"
        echo "  - NDK: ${NDK_PATH:-"(resolve later)"}"
    fi
    echo "  - Build Type: $BUILD_TYPE"
    echo "  - Clean Build: $CLEAN_BUILD"
    echo "  - Verbose: $VERBOSE"
    echo "  - Build Dir: $CPP_DIR/$BUILD_DIR"
    echo "  - Deps: Conan (conanfile.py)"
    if [ "$BUILD_TARGET" = "android" ]; then
        echo "  - jniLibs: $PROJECT_DIR/app/libs/$ABI"
        echo "  - runtime_deploy: $CPP_DIR/$BUILD_DIR/runtime_deploy"
    fi
    echo
}

# The shipped config is JSON with long prose comments, and a missing comma between two of them is an
# easy edit to make. `AdasApp::Config::loadFromFile` reacts by logging one line and falling back to
# built-in defaults — on the phone that means the drive silently runs on the wrong parameters. Cheaper
# to fail here. (It happened twice on 2026-08-06 alone.)
check_shipped_config() {
    local cfg
    cfg="$(dirname "$0")/../assets/config.json"
    [ -f "$cfg" ] || return 0
    if ! python3 -c "import json,sys;json.load(open(sys.argv[1]))" "$cfg" 2>/tmp/adas_cfg_err; then
        print_error "assets/config.json is not valid JSON — the app would silently use defaults:"
        cat /tmp/adas_cfg_err >&2
        exit 1
    fi
    print_success "assets/config.json parses"
}

# The course is read by people who copy the snippets, so a block that raises is worse than no block.
# Cheap to check here; skipped silently when numpy is absent (the C++ build does not need it).
check_book_snippets() {
    local checker
    checker="$(dirname "$0")/../../../../docs/book/check_snippets.py"
    [ -f "$checker" ] || return 0
    python3 -c "import numpy" 2>/dev/null || return 0
    if ! python3 "$checker" >/tmp/adas_book_snippets 2>&1; then
        print_error "a python block in docs/book does not run:"
        tail -20 /tmp/adas_book_snippets >&2
        exit 1
    fi
    print_success "$(tail -1 /tmp/adas_book_snippets)"
}

main() {
    echo "=========================================="
    echo "  ADAS C++ build (Conan)"
    echo "=========================================="
    echo

    BUILT_LIBRARY=""
    parse_arguments "$@"
    # One build/ root, split by target and build type: build/<target>/<BuildType>, e.g.
    # build/linux/Release or build/android/Debug. Conan and cmake both write absolute paths into
    # their caches, so a host build and a container build cannot share a directory — hence the
    # suffix on the target segment rather than a separate root.
    BUILD_DIR="build/${BUILD_TARGET}${ADAS_BUILD_DIR_SUFFIX:-}/${BUILD_TYPE}"
    show_build_info

    check_dependencies
    check_shipped_config
    check_book_snippets
    clean_build
    build_cpp_library
    copy_to_jnilibs

    echo
    print_success "C++ build finished successfully"
    if [ "$BUILD_TARGET" = "android" ]; then
        print_status "Library ready for the Android app"
    else
        print_status "Linux library ready"
    fi
}

main "$@"
