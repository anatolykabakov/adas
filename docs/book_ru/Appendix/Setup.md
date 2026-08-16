# Установка и запуск

## Репозиторий

Корень Android ADAS:

```bash
cd <корень репозитория>
```

Смотрите также `README.md` в корне репозитория: сборка APK, `run_bag_vis.sh`, `run_sim.sh`, раскладка Java / C++ / скриптов.

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

Кнопка **ENG/RU** сверху переводит на ту же страницу в другой языковой версии. Раскладка `_site` и `_site/ru`
— это контракт, на который опирается кнопка, а не деталь реализации; скрипт сборки отдельно проверяет, что у
каждой страницы есть пара, иначе переключатель приводил бы в 404.

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

Рекомендуемый порядок: обзор Lane Detection и Control/PP в AAD → эта книга (Архитектура → Управление → Задержки) → задания A1/B1.

## Телефон

* Android arm64, USB OTG и Panda;
* модель в `/sdcard/adas_models/supercombo.onnx` или в assets;
* логи → `/sdcard/adas_logs/…`;
* для учебных измерений: `vision_traffic: false`, `phone_stats: true`.

<!-- next-chapter -->
---

**Дальше:** [Что читать дальше](./ReadingList.md)
