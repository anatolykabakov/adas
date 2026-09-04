# Установка и запуск

## Репозиторий

Склонируйте и соберите — всё остальное одна команда сделает сама:

```bash
git clone https://github.com/anatolykabakov/adas.git
cd adas
./scripts/build_project.sh   # local.properties, модели, сборка APK
```

`<корень репозитория>` в остальной книге — это и есть каталог `adas/`.

**Что сборка ожидает уже установленным** (на чистой Linux/macOS это нужно поставить заранее —
на Debian/Ubuntu за вас это сделает `scripts/install_dependencies.sh`): JDK (17+), Android SDK и NDK
(`ANDROID_HOME` должен указывать на SDK, где уже есть платформа `compileSdk`), Conan 2 и CMake для C++,
Python 3.10+ с `numpy`. Всё офлайновое — разбор бегов, симулятор, `pyadas` — обходится одной Python-частью
и работает **без телефона и машины** (см. [Беги](../Logging/Bags.md), *Запишите собственный бег*, — это самый
короткий путь, не требующий железа).

Моделей в репозитории нет. `scripts/fetch_models.sh` (сборка запускает его сама) скачивает fp16-supercombo
из релиза comma и выводит из неё варианты fp32 и `.thneed`, сверяя каждый sha256 с
`scripts/models.manifest`. Запускайте его отдельно, если нужны только файлы.

См. также `README.md` в корне репозитория: сборка APK, `run_bag_vis.sh`, `run_sim.sh`, раскладка Java / C++ / скриптов.

## Сборка книги (HTML)

```bash
cd docs
python3 -m venv .venv && source .venv/bin/activate
pip install -r book/requirements.txt
./build_book.sh          # обе версии: EN в _site, RU в _site/ru
./build_book.sh --en     # только английская, быстрее при правках
./build_book.sh --check  # проверки без сборки
# Живой сайт: https://anatolykabakov.github.io/adas/ (CI deploy-book на main)
```

Кнопка **ENG/RU** сверху открывает ту же страницу в другой языковой версии. Раскладка `_site` и `_site/ru`
— это не деталь реализации, а контракт, на который опирается кнопка; скрипт сборки отдельно проверяет, что у
каждой страницы есть пара, — иначе переключатель уводил бы в 404.

## Python для бегов

```bash
cd scripts
# заглушки protobuf:
./generate_proto_python.sh
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
python3 tools/latency.py /путь/к/adas_logs/СЕССИЯ
```

## AAD (по желанию)

Для глав про камеру «с нуля» и интерактивных упражнений по Pure Pursuit в CARLA:

[Algorithms for Automated Driving](https://github.com/thomasfermi/Algorithms-for-Automated-Driving)

Рекомендуемый порядок: обзор Lane Detection и Control/PP в AAD → эта книга (Архитектура → Управление → Задержки) → задания A1 и B1.

## Телефон

* Android arm64, USB OTG и Panda;
* модели берутся из ассетов APK (`supercombo.onnx`, `supercombo.thneed`); каталог `/sdcard/adas_models/` перекрывает
  их, когда нужно проверить другой файл без пересборки;
* новый телефон: сначала `python3 tools/model_device_probe.py --iters 50` — см. `docs/NEW_PHONE.md`;
* логи пишутся в `/sdcard/adas_logs/…`;
* для учебных измерений: `vision_traffic: false`, `phone_stats: true`.

<!-- next-chapter -->
---

**Дальше:** [Что читать дальше](./ReadingList.md)
