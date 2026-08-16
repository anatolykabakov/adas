# clang-tidy: отчёт по файлам с приоритетами — 2026-08-15

Прогон: `clang-tidy-20`, конфиг `app/src/main/cpp/.clang-tidy`, 39 единиц трансляции
(`src/{services,utils,lateral,mapmatch,platform,panda}/*.cpp`). Вендорное (`thneed`, `json11`, `CL/`)
и сгенерированное (`acados_lat_ocp`, `*.pb.cc`) исключены. **Разобрались все 39** — до правки
`manager.hpp` тринадцать не разбирались вовсе.

Как запускать — полный прогон, режим «только по своим правкам», приоритеты и ловушки — описано в
`scripts/cpp/README.md`. Коротко, чтобы воспроизвести этот отчёт:

```bash
./build_cpp.sh -t linux --test            # заодно кладёт build/linux/Release/compile_commands.json
python3 ../../../../scripts/cpp/run-clang-tidy.py \
      -clang-tidy-binary /usr/bin/clang-tidy-20 -p build/linux/Release -config-file=.clang-tidy \
      -extra-arg=--gcc-install-dir=/usr/lib/gcc/x86_64-linux-gnu/11 \
      -export-fixes=tidy_errors.yml -quiet -j 6 \
      "src/(services|utils|lateral|mapmatch|platform|panda)/.*\.cpp"
python3 ../../../../scripts/cpp/tidy_report_by_file.py tidy_errors.yml .clang-tidy --root "$(pwd)/" --md
```

`--extra-arg=--gcc-install-dir` обязателен: clang выбирает GCC 12, для которого в системе нет
заголовков libstdc++, и без указания не находит даже `<cstdint>`.

Приоритеты: **P0** — может быть неверно во время работы (анализатор доказал путь, сужение типа теряет
значение, макрос из нескольких инструкций); **P1** — небезопасно по построению (границы, снятие const,
C-массивы, глобальные переменные); **P2** — косметика (скобки, имена, гигиена включений, модернизация).

Сортировка — по P0, затем P1, затем всего. Первые строки таблицы и есть план работ.

Findings: **1364** — P0 1, P1 135, P2 1228

| file | P0 | P1 | P2 | dominant check |
|---|---|---|---|---|
| `/home/anatoly/.conan2/p/cppzm0c71a752fad11/p/include/zmq.hpp` | 1 | 0 | 0 | clang-analyzer-optin.core.EnumCastOutOfRange (1) |
| `src/lateral/flowpilot_mpc.cpp` | 0 | 37 | 54 | cppcoreguidelines-pro-bounds-constant-array-index (36) |
| `src/services/zmq_bridge.cpp` | 0 | 13 | 56 | misc-include-cleaner (50) |
| `src/platform/volkswagen/panda_safety_supervisor.cpp` | 0 | 12 | 13 | cppcoreguidelines-pro-type-vararg (12) |
| `src/platform/volkswagen/mqbcan.cpp` | 0 | 9 | 31 | readability-identifier-naming (10) |
| `src/lateral/acados_lat_mpc.cpp` | 0 | 9 | 19 | google-readability-braces-around-statements (9) |
| `src/services/control.cpp` | 0 | 6 | 26 | misc-include-cleaner (12) |
| `src/services/map_data.cpp` | 0 | 6 | 24 | misc-include-cleaner (13) |
| `src/mapmatch/road_map.cpp` | 0 | 5 | 64 | google-readability-braces-around-statements (20) |
| `src/services/platform.cpp` | 0 | 5 | 25 | misc-include-cleaner (12) |
| `src/utils/pose_calibrator.cpp` | 0 | 5 | 22 | google-readability-braces-around-statements (10) |
| `src/services/planner.cpp` | 0 | 3 | 48 | misc-include-cleaner (34) |
| `src/mapmatch/search.cpp` | 0 | 3 | 46 | google-readability-braces-around-statements (30) |
| `src/services/localization.cpp` | 0 | 3 | 34 | misc-include-cleaner (17) |
| `src/mapmatch/fit.cpp` | 0 | 2 | 86 | google-readability-braces-around-statements (36) |
| `src/mapmatch/window_search.cpp` | 0 | 2 | 31 | google-readability-braces-around-statements (11) |
| `src/utils/adas_config.cpp` | 0 | 2 | 20 | google-readability-braces-around-statements (11) |
| `src/services/middleware_stats.cpp` | 0 | 2 | 8 | google-readability-braces-around-statements (3) |
| `src/platform/volkswagen/car_iface.cpp` | 0 | 2 | 6 | misc-include-cleaner (5) |
| `src/mapmatch/road_route.cpp` | 0 | 1 | 75 | google-readability-braces-around-statements (35) |
| `src/utils/vehicle_ekf.cpp` | 0 | 1 | 49 | readability-identifier-naming (27) |
| `src/services/safety_warn.cpp` | 0 | 1 | 22 | misc-include-cleaner (17) |
| `src/lateral/kappa_solver.cpp` | 0 | 1 | 19 | misc-include-cleaner (9) |
| `src/platform/volkswagen/mqb_car_state_decoder.cpp` | 0 | 1 | 17 | misc-include-cleaner (7) |
| `src/services/traffic_sign.cpp` | 0 | 1 | 17 | misc-include-cleaner (12) |
| `src/services/camera_calib.cpp` | 0 | 1 | 15 | misc-include-cleaner (12) |
| `src/utils/imu_calibrator.cpp` | 0 | 1 | 14 | google-readability-braces-around-statements (10) |
| `include/adas/lateral/visionpilot_mpc.h` | 0 | 1 | 0 | performance-trivially-destructible (1) |
| `src/utils/proto_convert.cpp` | 0 | 0 | 72 | misc-include-cleaner (50) |
| `src/mapmatch/track.cpp` | 0 | 0 | 70 | google-readability-braces-around-statements (44) |
| `src/utils/lane_path.cpp` | 0 | 0 | 51 | google-readability-braces-around-statements (25) |
| `src/utils/path_lateral_state.cpp` | 0 | 0 | 31 | readability-math-missing-parentheses (13) |
| `src/lateral/visionpilot_mpc.cpp` | 0 | 0 | 30 | readability-identifier-naming (13) |
| `src/utils/vanishing_point_calib.cpp` | 0 | 0 | 24 | google-readability-braces-around-statements (12) |
| `src/lateral/pp_planner.cpp` | 0 | 0 | 20 | google-readability-braces-around-statements (8) |
| `src/lateral/fp_planner.cpp` | 0 | 0 | 19 | misc-include-cleaner (13) |
| `src/platform/volkswagen/carcontroller.cpp` | 0 | 0 | 18 | misc-include-cleaner (10) |
| `src/utils/online_localizer.cpp` | 0 | 0 | 18 | google-readability-braces-around-statements (7) |
| `src/lateral/vp_planner.cpp` | 0 | 0 | 18 | misc-include-cleaner (6) |
| `src/services/internal_subscriber.cpp` | 0 | 0 | 13 | misc-include-cleaner (12) |
| `include/adas/services/localization.h` | 0 | 0 | 1 | readability-inconsistent-declaration-parameter-name (1) |
| `src/utils/lane_keep_gates.cpp` | 0 | 0 | 1 | misc-include-cleaner (1) |
| `include/adas/services/planner.h` | 0 | 0 | 1 | readability-inconsistent-declaration-parameter-name (1) |

## Что исправлено

P0 было 43, стало 1 — и единственный оставшийся лежит в `zmq.hpp` из conan, то есть не в нашем коде.

| правка | закрыто | что это было |
|---|---|---|
| `include/adas/utils/logger.h` — `do { … } while (0)` вокруг пяти линуксовых макросов | 8 | настоящий дефект: `if (cond) LOGW(...)` без скобок печатал перевод строки и делал `fflush` **всегда**, мимо условия |
| `proto_convert.cpp` — 11 явных `static_cast<float>` | 11 | поля прото объявлены `float`, источники — `double`; потеря точности теперь видна в коде |
| `track.cpp`, `fit.cpp` — `std::ptrdiff_t` / `Eigen::Index` на итераторах и индексах | 9 | `begin() + n` с беззнаковым `n`; знаковый тип теперь берётся явно |
| `localization.cpp`, `map_data.cpp`, `mqb_car_state_decoder.cpp` — `static_cast<double>` перед умножением на `1e-3`/`1e-6` | 5 | целые метки времени неявно уходили в `double` |
| `acados_lat_mpc.cpp` — `kCostDim`/`kCostDimE` стали `std::size_t` | 2 | индекс `i * kCostDim + i` считался в `int` и расширялся при обращении к массиву |
| `flowpilot_mpc.cpp` — `static_cast<size_t>(N) + 1` вместо `static_cast<size_t>(N + 1)` | 2 | сложение шло в `int`, расширение — после него |
| `road_map.cpp` — счётчик колец стал целым, радиус через `std::ldexp` | 2 | `for (double r = …; r *= 2.0)`; удвоение точное, но счётчик цикла был вещественным |
| `fit.cpp:328` — ветка `c == 2` поглощена веткой `c < 5` | 1 | обе ставили `scale = 1e-3` |
| `track.cpp:134` — снято мёртвое присваивание `n` | 1 | значение дальше не читалось |
| `carcontroller.cpp:67` — `1.9 * (100.0 / STEER_STEP)` | 1 | целочисленное деление в вещественном выражении; при `STEER_STEP = 2` результат тот же, но запись теперь честная |

В `.clang-tidy` добавлено `cppcoreguidelines-avoid-do-while.IgnoreMacros: true`: обёртка `do/while` —
единственный способ сделать многоинструкционный макрос одной инструкцией, и запрещать её там же, где
она обязательна, смысла нет. Это сняло 64 P1, появившихся ровно от правильной правки.

Сборка после всех правок зелёная, `223/223` тестов проходят.
