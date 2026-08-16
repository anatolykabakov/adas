#!/bin/bash
# Подготовить телефон к выезду: установить APK, выставить конфиг заезда, проверить результат.
#
# Одна команда вместо трёх шагов, потому что порядок важен: конфиг читается на старте приложения,
# а установка APK его не перезаписывает (AdasAppHandler.ensureAssetCopied, force=false).
#
#   ./scripts/prepare_drive.sh              # показать, что будет сделано
#   ./scripts/prepare_drive.sh --apply
#
# Что именно ставится в конфиг и почему — см. ниже, каждый ключ с обоснованием из замеров.

set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
APK="${PROJECT_DIR}/app/build/outputs/apk/debug/app-debug.apk"
APPLY=false
[ "${1:-}" = "--apply" ] && APPLY=true
# Правки этого заезда. Держать в согласии с docs/PREDRIVE.md — там же критерий, по которому заезд
# читается. Разошлись — значит на дорогу поедет прошлый эксперимент под именем нового.
SETS=(
  "vision.model_runner=thneed"
  "localization.use_camera_odometry=false"
  "vehicle.lat_pid_kf=6e-05"
  "vehicle.lat_pid_ff_floor_mps=0.0"
  "vehicle.fp_solver=acados"
  "vehicle.tire_stiffness_factor=1.0"
  "localization.params_stiffness_p0_std=0.05"
)

echo "=== проверка"
adb get-state >/dev/null 2>&1 || { echo "телефон не подключён (adb devices)" >&2; exit 1; }
[ -f "$APK" ] || { echo "нет APK: $APK — соберите ./scripts/docker.sh apk" >&2; exit 1; }

# Gradle нативную библиотеку НЕ собирает: она приходит готовой из app/libs/arm64-v8a/ через
# app/src/main/cpp/build_cpp.sh, а assembleDebug отрабатывает за три секунды и упаковывает то, что
# лежит. 2026-08-09 так и уехала бы сборка с библиотекой трёхчасовой давности, без правки
# упреждения целиком. Сверяем не даты, а содержимое: sha1 .so внутри APK против собранной.
check_native() {
    local lib="${PROJECT_DIR}/app/libs/arm64-v8a/libadas_app_android.so"
    [ -f "$lib" ] || { echo "  нативной библиотеки нет: $lib" >&2; return 1; }

    local newest
    newest="$(find "${PROJECT_DIR}/app/src/main/cpp" -name '*.cpp' -o -name '*.h' -o -name '*.hpp' \
        | grep -vE '/build-|/tests/' | xargs ls -t 2>/dev/null | head -1)"
    if [ -n "$newest" ] && [ "$newest" -nt "$lib" ]; then
        echo "  ИСХОДНИКИ C++ НОВЕЕ БИБЛИОТЕКИ: $(basename "$newest")" >&2
        echo "  соберите: (cd app/src/main/cpp && ./build_cpp.sh -t android) && ./gradlew :app:assembleDebug" >&2
        return 1
    fi

    local in_apk built
    in_apk="$(unzip -p "$APK" lib/arm64-v8a/libadas_app_android.so 2>/dev/null | sha1sum | cut -d' ' -f1)"
    built="$(sha1sum "$lib" | cut -d' ' -f1)"
    if [ "$in_apk" != "$built" ]; then
        echo "  В APK ДРУГАЯ БИБЛИОТЕКА: $in_apk против $built" >&2
        echo "  пересоберите APK: ./gradlew :app:assembleDebug" >&2
        return 1
    fi
    echo "  нативная:   ${built:0:12} — совпадает с APK"
}
check_native || exit 1
echo "  устройство: $(adb shell getprop ro.product.model | tr -d '\r')"
echo "  APK:        $(du -h "$APK" | cut -f1), $(date -r "$APK" '+%Y-%m-%d %H:%M')"

if ! $APPLY; then
  echo
  echo "=== будет сделано (это показ, ничего не меняется)"
  echo "  1) adb install -r <APK>"
  for s in "${SETS[@]}"; do echo "  2) конфиг: $s"; done
  echo "  3) чтение конфига с устройства для проверки"
  echo
  "${PROJECT_DIR}/scripts/push_config.sh" "${SETS[@]/#/--set }" 2>/dev/null || true
  echo "применить: ./scripts/prepare_drive.sh --apply"
  exit 0
fi

echo
echo "=== установка APK"
adb install -r "$APK" 2>&1 | tail -1

echo
echo "=== конфиг"
args=()
for s in "${SETS[@]}"; do args+=(--set "$s"); done
"${PROJECT_DIR}/scripts/push_config.sh" --apply "${args[@]}"

echo
echo "=== проверка на устройстве"
adb shell run-as adas.app cat /data/data/adas.app/files/config.json | python3 -c '
import json, sys
c = json.load(sys.stdin)
cam, veh = c["calibration"]["camera"], c["vehicle"]
rows = [
    ("rpy_deg", cam.get("rpy_deg")),
    ("intrinsics fx/fy", (cam["intrinsics_prior"]["fx"], cam["intrinsics_prior"]["fy"])),
    ("tire_stiffness_factor", veh.get("tire_stiffness_factor")),
    ("steer_ratio", veh.get("steer_ratio")),
    ("path_lane_blend_scale", veh.get("path_lane_blend_scale")),
    ("lane_std_good/bad_m", (veh.get("lane_std_good_m"), veh.get("lane_std_bad_m"))),
    ("lane_max_age_s", veh.get("lane_max_age_s")),
    ("lka_suppress_on_blinker", veh.get("lka_suppress_on_blinker")),
    ("lane_keep_controller", veh.get("lane_keep_controller")),
    ("-- пакет паритета с dp", ""),
    ("lane_std good/bad", (veh.get("lane_std_good_m"), veh.get("lane_std_bad_m"))),
    ("path_lane_blend_scale", veh.get("path_lane_blend_scale")),
    ("lane_mode_hysteresis", veh.get("lane_mode_hysteresis")),
    ("roll_compensation", veh.get("roll_compensation")),
    ("use_learned_params", veh.get("use_learned_params")),
    ("fp_solver", veh.get("fp_solver")),
    ("dp_parity_pack", veh.get("dp_parity_pack")),
    ("record_camera_images", c.get("logging", {}).get("record_camera_images")),
]
for k, v in rows:
    print(f"  {k:24s} {v}")
'

cat <<'PLAN'

=== в машине
  1) телефон на зарядке: в беге 08-04 заряд ушёл с 27 % до 4 %;
  2) маршрут тот же, что 2026_08_06_00_36_42 — нужны те же дуги, иначе сравнивать нечего: у 08-04 и
     08-06 радиусы правых дуг различались (130–273 м против 71–134 м), и это само по себе сдвинуло
     метрику сильнее, чем любая правка;
  3) логирование включить до начала движения;
  4) первую минуту не ждать от ассистента точности: калибровка сходится 30–60 с;
  5) на длинной дуге переключить слайдер «Lane blend» 0.6 → 1.0 и обратно: это даёт сравнение
     внутри одного бега, а перезапуск приложения между заездами меняет калибровку и полосу.

  Если включаете кнопки круиза (см. выше, отдельным решением): план работает только при включённом
  штатном круизе, нажимает +/− шагом 1 км/ч не чаще раза в 200 мс и только пока план свежее 500 мс.
  Уставка, выбранная водителем, теперь потолок — ассистент может только отдать скорость и вернуть её.
  Педали и отключение круиза перебивают его сразу. Что смотреть: нажатий должно быть единицы в
  минуту, а не десятки; замедление доступно только выбегом (около −0.3 м/с²), всё сильнее уходит в
  статус `brake_needed` — это просьба к водителю, а не команда.

=== что проверять после
  ./scripts/bag/bag_arc_offset.py <бег> --blend <из лога> --shift 0.05 \
      --std-good 0.3 --std-bad 1.5 --width-max 4.6 --weight-by-std
  * левая дуга: было +0.23 при tsf 0.50, ошибка слежения +0.21;
  * правые дуги: смотреть не итог, а колонку σ worst и d — при σ > 1.5 подмешивание отключается и
    эталоном становится план модели, тогда итог говорит о перцепции, а не о контроллере;
  * насыщение момента на правых дугах: 65 % кадров, не должно расти;
  * предупреждения: FCW/AEB ниже 8 м/с должны замолчать (в 08-06 гейт не работал — в конфиге
    оставалось 3.0), LDW уже молчит;
  * поворотник: в логе статус `blinker` и снятая команда на каждом перестроении;
  * темп зрения: `latency.py <бег>` — на 08-06 получилось 13.24 Гц и 79 мс до CAN. В новых бегах там же
    появятся строки delivery_ms (камера → приложение), queue_ms (ожидание своей очереди) и
    frames_dropped: много потерь при коротком delivery — инференс не успевает, мало потерь при долгом
    delivery — кадр приходит поздно;
  * продольный план: `bag_long_replay.py <бег>` — нажатий в минуту и доля `brake_needed`.
PLAN
