# Статический анализ: clang-tidy

Четыре скрипта. Первые два запускают анализ — по всему коду или только по изменённым строкам, — два
других превращают выгрузку clang-tidy в читаемый отчёт.

| скрипт | что делает |
|---|---|
| `run-clang-tidy.py` | апстримный из LLVM: гоняет clang-tidy по всей базе компиляции в несколько потоков |
| `clang_tidy_diff.py` | апстримный `clang-tidy-diff.py` из LLVM: читает unified diff и проверяет только затронутые строки |
| `tidy_report_handler.py` | YAML из `-export-fixes` → JSON по каждой находке плюс сводка по проверкам; код возврата 1, если что-то нашлось |
| `tidy_report_by_file.py` | тот же YAML → таблица по файлам с приоритетами P0/P1/P2; отвечает на вопрос «с чего начинать» |

Оба отчётных скрипта фильтруют находки тем же `Checks` из `.clang-tidy`, что и сам clang-tidy, поэтому
между собой они никогда не расходятся.

## Требования

* `clang-tidy-20` (`/usr/bin/clang-tidy-20`). В образе `.devcontainer/Dockerfile` его **нет** — на
  машине разработчика ставится отдельно: `sudo apt install clang-tidy-20`.
* `compile_commands.json`. Отдельно генерировать не нужно: `CMAKE_EXPORT_COMPILE_COMMANDS` включён в
  `CMakeLists.txt`, так что база появляется после любой сборки.
* `python3 -m pip install pyyaml` — оба отчётных скрипта читают YAML.

## Полный прогон

Все команды — из `app/src/main/cpp`: там лежат `.clang-tidy` и база компиляции. Сами скрипты — в
корне репозитория, отсюда переменная `TIDY`.

```bash
TIDY=../../../../scripts/cpp

# 1. Собрать: заодно кладёт build/linux/Release/compile_commands.json
./build_cpp.sh -t linux --test

# 2. Прогнать анализ (~5 минут на 14 потоках)
python3 $TIDY/run-clang-tidy.py \
      -clang-tidy-binary /usr/bin/clang-tidy-20 -p build/linux/Release -config-file=.clang-tidy \
      -extra-arg=--gcc-install-dir=/usr/lib/gcc/x86_64-linux-gnu/11 \
      -export-fixes=tidy_errors.yml -quiet -j "$(nproc)" \
      "src/(services|utils|lateral|mapmatch|platform|panda)/.*\.cpp"

# 3. Отчёты
python3 $TIDY/tidy_report_by_file.py tidy_errors.yml .clang-tidy --root "$(pwd)/" --md
python3 $TIDY/tidy_report_handler.py tidy_errors.yml .clang-tidy > clang_errors.txt
```

`run-clang-tidy.py` и `clang_tidy_diff.py` — копии апстримных скриптов LLVM (Apache 2.0 with LLVM
exceptions, заголовок в файлах сохранён), лежат здесь без единой правки, чтобы прогон не зависел от
соседних репозиториев и обновлялся простой заменой файла. Дистрибутивные `run-clang-tidy-20` и
`clang-tidy-diff-20.py` принимают те же ключи и годятся как замена; отличаются они только
умолчаниями для путей к бинарям, а мы задаём их явно.

Регулярка в конце ограничивает охват нашим кодом. Без неё в отчёт попадают `src/lateral/acados_lat_ocp/*.c`
(генерируется acados), `src/panda/*.cc` и `src/utils/can_parser.cc` (взяты из openpilot/opendbc) —
править их бессмысленно, а находок они добавляют около сорока.

## Только по своим правкам

Ежедневный режим: анализ идёт лишь по строкам, которые вы тронули. Секунды вместо минут.

```bash
TIDY=../../../../scripts/cpp
git -C ../../../.. diff -U0 --relative=app/src/main/cpp -- '*.cpp' \
  | python3 $TIDY/clang_tidy_diff.py \
      -clang-tidy-binary /usr/bin/clang-tidy-20 -config-file=.clang-tidy \
      -path build/linux/Release -p1 -j "$(nproc)" -only-in-compile-db -quiet \
      -extra-arg=--gcc-install-dir=/usr/lib/gcc/x86_64-linux-gnu/11 \
      -export-fixes=tidy_diff.yml
python3 $TIDY/tidy_report_by_file.py tidy_diff.yml .clang-tidy --root "$(pwd)/"
```

`--relative=app/src/main/cpp` вместе с `-p1` дают пути относительно текущего каталога — иначе имена из
diff не совпадут с записями в базе компиляции. `-only-in-compile-db` отбрасывает заголовки и всё, чего
в базе нет.

Так выглядит вывод на реальном наборе правок:

```
findings: 13  (P0 0, P1 4, P2 9)

file                                                     P0   P1    P2  dominant check
src/lateral/acados_lat_mpc.cpp                            0    4     2  cppcoreguidelines-pro-bounds-constant-array-index (4)
src/mapmatch/road_map.cpp                                 0    0     2  google-readability-braces-around-statements (2)
```

## Приоритеты

`tidy_report_by_file.py` раскладывает проверки по трём уровням — сортировка идёт по P0, затем P1,
затем по общему числу, так что первые строки таблицы и есть план работ.

* **P0 — может быть неверно во время работы.** Файл вообще не разобрался, анализатор доказал путь,
  сужение типа теряет значение, макрос из нескольких инструкций. Это меняет поведение, а не вид кода.
* **P1 — небезопасно по построению.** Границы, снятие `const`, C-массивы, глобальные переменные:
  сегодня ничего не сломано, но гарантии нет, и счёт приходит при следующей правке.
* **P2 — косметика.** Скобки, имена, гигиена включений, модернизация. Один `misc-include-cleaner` даёт
  сотни находок, и каждая правка — строчка `#include`.

Раскладку задаёт словарь `TIERS` в `tidy_report_by_file.py`: выигрывает самый длинный совпавший
префикс, поэтому отдельную проверку можно поднять или опустить относительно её семейства.

## Ловушки

**`--extra-arg=--gcc-install-dir` обязателен.** clang выбирает GCC 12, заголовков libstdc++ для
которого в системе нет, и без указания каталога не находит даже `<cstdint>`. Проверить свой набор:
`ls /usr/lib/gcc/x86_64-linux-gnu/`.

**В `Checks` выигрывает последнее совпадение.** Список разбирается по запятым, `-` исключает, и семья,
названная второй раз, молча возвращает всё, что исключено выше. Поэтому в `.clang-tidy` сначала идут
все семейства, затем все исключения, по одной записи на строку. Проверить конфиг:

```bash
clang-tidy-20 --config-file=.clang-tidy --verify-config
```

**`WarningsAsErrors: '*'`.** Сейчас в отчёте больше тысячи находок P2, так что включать этот режим в
блокирующем виде рано — сборку он остановит на первой же скобке.

**Отчёты не совпадают с сырым выводом clang-tidy.** Скрипты отбрасывают чужие заголовки и тестовые
макросы (`IGNORE_LIST` в `tidy_report_handler.py`), поэтому «125845 warnings generated» от самого
clang-tidy и десяток строк в отчёте — это не противоречие.

## CI

Задачи в `.github/workflows/ci.yml` пока нет: в образе `adas-tools:ci` не установлен clang-tidy.
Чтобы завести, нужно добавить пакет в `.devcontainer/Dockerfile` и шаг после сборки — по образцу
`adas_dev_calibrations/.gitlab-ci.yml`, где проверяются только строки merge request:

```yaml
      - name: clang-tidy (changed lines)
        working-directory: app/src/main/cpp
        run: |
          git fetch origin main
          git diff -U0 origin/main --relative=app/src/main/cpp -- '*.cpp' \
            | python3 ../../../../scripts/cpp/clang_tidy_diff.py -clang-tidy-binary /usr/bin/clang-tidy-20 \
                -config-file=.clang-tidy -path build/linux/Release -p1 -j "$(nproc)" \
                -only-in-compile-db -export-fixes=tidy_errors.yml || true
          python3 ../../../../scripts/cpp/tidy_report_handler.py tidy_errors.yml .clang-tidy > clang_errors.txt
```

`tidy_report_handler.py` возвращает 1, когда есть находки, — этого достаточно, чтобы задача падала;
`clang_errors.txt` стоит сохранять артефактом.

## Артефакты

`tidy_errors.yml` и `clang_errors.txt` — результаты прогона, в git они не нужны. Последний разбор
лежит в `docs/archive/CLANG_TIDY_2026_08_15.md`.
