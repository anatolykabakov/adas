#include "panda/panda_comms.h"

#include <cassert>
#include <stdexcept>

#include "utils/logger.h"

#define USB_CTRL_TIMEOUT_MS 100

#define USB_MAX_RETRIES 5

static bool is_fatal_usb_error(int err)
{
  switch (err) {
    case LIBUSB_ERROR_IO:
    case LIBUSB_ERROR_NO_DEVICE:
    case LIBUSB_ERROR_NOT_FOUND:
    case LIBUSB_ERROR_PIPE:
    case LIBUSB_ERROR_NO_MEM:
    case LIBUSB_ERROR_OTHER:
      return true;
    default:
      return false;
  }
}

static int init_usb_ctx(libusb_context** context)
{
  assert(context != nullptr);

#ifdef LIBUSB_OPTION_WEAK_AUTHORITY
  libusb_set_option(NULL, LIBUSB_OPTION_WEAK_AUTHORITY);
#endif
  int err = libusb_init(context);
  if (err != 0) {
    LOGE("libusb_init failed: %d %s", err, libusb_strerror((enum libusb_error)err));
    return err;
  }

#if LIBUSB_API_VERSION >= 0x01000106
  libusb_set_option(*context, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_WARNING);
#else
  libusb_set_debug(*context, 2);
#endif

  return err;
}

PandaUsbHandle::PandaUsbHandle(std::string serial) : PandaCommsHandle(serial)
{
  ssize_t num_devices;
  libusb_device** dev_list = NULL;
  int err = init_usb_ctx(&ctx);
  if (err != 0) {
    goto fail;
  }

  num_devices = libusb_get_device_list(ctx, &dev_list);
  if (num_devices < 0) {
    goto fail;
  }
  for (size_t i = 0; i < num_devices; ++i) {
    libusb_device_descriptor desc;
    libusb_get_device_descriptor(dev_list[i], &desc);
    if (desc.idVendor == 0xbbaa && desc.idProduct == 0xddcc) {
      int ret = libusb_open(dev_list[i], &dev_handle);
      if (dev_handle == NULL || ret < 0) {
        goto fail;
      }

      unsigned char desc_serial[26] = {0};
      ret = libusb_get_string_descriptor_ascii(dev_handle, desc.iSerialNumber, desc_serial, std::size(desc_serial));
      if (ret < 0) {
        goto fail;
      }

      hw_serial = std::string((char*)desc_serial, ret);
      if (serial.empty() || serial == hw_serial) {
        break;
      }
      libusb_close(dev_handle);
      dev_handle = NULL;
    }
  }
  if (dev_handle == NULL)
    goto fail;
  libusb_free_device_list(dev_list, 1);
  dev_list = nullptr;

  if (libusb_kernel_driver_active(dev_handle, 0) == 1) {
    libusb_detach_kernel_driver(dev_handle, 0);
  }

  err = libusb_set_configuration(dev_handle, 1);
  if (err != 0) {
    goto fail;
  }

  err = libusb_claim_interface(dev_handle, 0);
  if (err != 0) {
    goto fail;
  }

  return;

fail:
  if (dev_list != NULL) {
    libusb_free_device_list(dev_list, 1);
  }
  cleanup();
  throw std::runtime_error("Error connecting to panda");
}

PandaUsbHandle::PandaUsbHandle(int fd) : PandaCommsHandle(fd)
{
  libusb_device* device = NULL;
  int ret;
  int err = 0;

#ifdef LIBUSB_OPTION_NO_DEVICE_DISCOVERY
  libusb_set_option(NULL, LIBUSB_OPTION_NO_DEVICE_DISCOVERY, NULL);
#endif
  err = init_usb_ctx(&ctx);
  if (err != 0) {
    goto fail;
  }

  ret = libusb_wrap_sys_device(ctx, (intptr_t)fd, &dev_handle);
  if (ret != 0 || dev_handle == NULL) {
    LOGE("libusb_wrap_sys_device(fd=%d) failed: %d %s", fd, ret, libusb_strerror((enum libusb_error)ret));
    goto fail;
  }
  LOGI("libusb_wrap_sys_device ok fd=%d", fd);

  device = libusb_get_device(dev_handle);
  libusb_device_descriptor desc;
  libusb_get_device_descriptor(device, &desc);
  if (desc.idVendor == 0xbbaa && desc.idProduct == 0xddcc) {
    unsigned char desc_serial[26] = {0};
    ret = libusb_get_string_descriptor_ascii(dev_handle, desc.iSerialNumber, desc_serial, std::size(desc_serial));
    if (ret < 0) {
      LOGE("get_string_descriptor_ascii failed: %d", ret);
      goto fail;
    }
    hw_serial = std::string((char*)desc_serial, ret).c_str();
    LOGI("Panda serial=%s VID=0x%04x PID=0x%04x", hw_serial.c_str(), desc.idVendor, desc.idProduct);
  } else {
    LOGW("Wrapped USB device is not panda VID=0x%04x PID=0x%04x", desc.idVendor, desc.idProduct);
  }

  if (libusb_kernel_driver_active(dev_handle, 0) == 1) {
    libusb_detach_kernel_driver(dev_handle, 0);
  }

#if defined(BUILD_FOR_ANDROID) || defined(__ANDROID__)
  LOGI("Android FD path: skip set_configuration, claim interface 0");
#else
  err = libusb_set_configuration(dev_handle, 1);
  if (err != 0 && err != LIBUSB_ERROR_BUSY && err != LIBUSB_ERROR_NOT_SUPPORTED) {
    LOGE("set_configuration failed: %d %s", err, libusb_strerror((enum libusb_error)err));
    goto fail;
  }
#endif

  err = libusb_claim_interface(dev_handle, 0);
  if (err != 0) {
    LOGE("claim_interface failed: %d %s", err, libusb_strerror((enum libusb_error)err));
    goto fail;
  }
  LOGI("USB claim_interface(0) ok");

  return;

fail:
  cleanup();
  throw std::runtime_error("Error connecting to panda");
}

PandaUsbHandle::~PandaUsbHandle()
{
  std::lock_guard lk(hw_lock);
  cleanup();
  connected = false;
}

void PandaUsbHandle::cleanup()
{
  if (dev_handle) {
    libusb_release_interface(dev_handle, 0);
    libusb_close(dev_handle);
  }

  if (ctx) {
    libusb_exit(ctx);
  }
}

std::vector<std::string> PandaUsbHandle::list()
{
  ssize_t num_devices;
  libusb_context* context = NULL;
  libusb_device** dev_list = NULL;
  std::vector<std::string> serials;

  int err = init_usb_ctx(&context);
  if (err != 0) {
    return serials;
  }

  num_devices = libusb_get_device_list(context, &dev_list);
  if (num_devices < 0) {
    goto finish;
  }
  printf("=== DEBUG: Found %zd USB devices ===\n", num_devices);
  for (size_t i = 0; i < num_devices; ++i) {
    libusb_device* device = dev_list[i];
    libusb_device_descriptor desc;
    libusb_get_device_descriptor(device, &desc);

    printf("Device %zu: VID=0x%04x, PID=0x%04x, Class=%d, SubClass=%d, Protocol=%d\n", i, desc.idVendor, desc.idProduct,
           desc.bDeviceClass, desc.bDeviceSubClass, desc.bDeviceProtocol);

    if (desc.idVendor == 0xbbaa && desc.idProduct == 0xddcc) {
      printf("*** Found Panda device! VID=0x%04x, PID=0x%04x ***\n", desc.idVendor, desc.idProduct);
      libusb_device_handle* handle = NULL;
      int ret = libusb_open(device, &handle);
      if (ret < 0) {
        goto finish;
      }

      unsigned char desc_serial[26] = {0};
      ret = libusb_get_string_descriptor_ascii(handle, desc.iSerialNumber, desc_serial, std::size(desc_serial));
      libusb_close(handle);
      if (ret < 0) {
        goto finish;
      }

      serials.push_back(std::string((char*)desc_serial, ret).c_str());
      printf("*** Added Panda serial: %s ***\n", desc_serial);
    }
  }
  printf("=== DEBUG: Total Panda devices found: %zu ===\n", serials.size());

finish:
  if (dev_list != NULL) {
    libusb_free_device_list(dev_list, 1);
  }
  if (context) {
    libusb_exit(context);
  }
  return serials;
}

void PandaUsbHandle::handle_usb_issue(int err, const char func[])
{
  LOGE("usb error %d \"%s\" in %s", err, libusb_strerror((enum libusb_error)err), func);
  if (is_fatal_usb_error(err)) {
    LOGE("lost connection (fatal usb error %d in %s)", err, func);
    connected = false;
    comms_healthy = false;
  }
}

int PandaUsbHandle::control_write(uint8_t bRequest, uint16_t wValue, uint16_t wIndex, unsigned int timeout)
{
  int err;
  int attempts = 0;
  const uint8_t bmRequestType = LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE;
  if (timeout == 0) {
    timeout = USB_CTRL_TIMEOUT_MS;
  }

  if (!connected) {
    return LIBUSB_ERROR_NO_DEVICE;
  }

  std::lock_guard lk(hw_lock);
  do {
    err = libusb_control_transfer(dev_handle, bmRequestType, bRequest, wValue, wIndex, NULL, 0, timeout);
    if (err < 0) {
      handle_usb_issue(err, __func__);
      ++attempts;
    }
  } while (err < 0 && connected && attempts < USB_MAX_RETRIES);

  if (err < 0 && connected) {
    LOGE("control_write giving up after %d retries (last err=%d)", attempts, err);
    connected = false;
    comms_healthy = false;
  }

  return err;
}

int PandaUsbHandle::control_read(uint8_t bRequest, uint16_t wValue, uint16_t wIndex, unsigned char* data,
                                 uint16_t wLength, unsigned int timeout)
{
  int err;
  int attempts = 0;
  const uint8_t bmRequestType = LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE;
  if (timeout == 0) {
    timeout = USB_CTRL_TIMEOUT_MS;
  }

  if (!connected) {
    return LIBUSB_ERROR_NO_DEVICE;
  }

  std::lock_guard lk(hw_lock);
  do {
    err = libusb_control_transfer(dev_handle, bmRequestType, bRequest, wValue, wIndex, data, wLength, timeout);
    if (err < 0) {
      handle_usb_issue(err, __func__);
      ++attempts;
    }
  } while (err < 0 && connected && attempts < USB_MAX_RETRIES);

  if (err < 0 && connected) {
    LOGE("control_read giving up after %d retries (last err=%d)", attempts, err);
    connected = false;
    comms_healthy = false;
  }

  return err;
}

int PandaUsbHandle::bulk_write(unsigned char endpoint, unsigned char* data, int length, unsigned int timeout)
{
  int err;
  int transferred = 0;
  int attempts = 0;

  if (!connected) {
    return 0;
  }

  std::lock_guard lk(hw_lock);
  do {
    err = libusb_bulk_transfer(dev_handle, endpoint, data, length, &transferred, timeout);

    if (err == LIBUSB_ERROR_TIMEOUT) {
      break;
    } else if (err != 0 || length != transferred) {
      handle_usb_issue(err, __func__);
      ++attempts;
    }
  } while (err != 0 && connected && attempts < USB_MAX_RETRIES);

  if (err != 0 && err != LIBUSB_ERROR_TIMEOUT && connected) {
    LOGE("bulk_write giving up after %d retries (last err=%d)", attempts, err);
    connected = false;
    comms_healthy = false;
  }

  return transferred;
}

int PandaUsbHandle::bulk_read(unsigned char endpoint, unsigned char* data, int length, unsigned int timeout)
{
  int err;
  int transferred = 0;
  int attempts = 0;

  if (!connected) {
    return 0;
  }

  std::lock_guard lk(hw_lock);

  do {
    err = libusb_bulk_transfer(dev_handle, endpoint, data, length, &transferred, timeout);

    if (err == LIBUSB_ERROR_TIMEOUT) {
      break;
    } else if (err == LIBUSB_ERROR_OVERFLOW) {
      comms_healthy = false;

      break;
    } else if (err != 0) {
      handle_usb_issue(err, __func__);
      ++attempts;
    }

  } while (err != 0 && connected && attempts < USB_MAX_RETRIES);

  if (err != 0 && err != LIBUSB_ERROR_TIMEOUT && err != LIBUSB_ERROR_OVERFLOW && connected) {
    LOGE("bulk_read giving up after %d retries (last err=%d)", attempts, err);
    connected = false;
    comms_healthy = false;
  }

  return transferred;
}
