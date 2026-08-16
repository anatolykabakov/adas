
#include <set>
#include <cstdio>
#include <jni.h>
#include <fstream>
#include <string>
#include <sstream>
#include <android/log.h>
#include <dlfcn.h>
#include <sys/mman.h>
#include <cassert>
#include <cerrno>
#include <cstring>
#include <map>

#include "adas/thneed/thneedmodel.h"
#include "adas/thneed/clutil.h"
#include "adas/thneed/timing.h"
#include "adas/thneed/json11.hpp"
#include "adas/thneed/util.h"
#include <dlfcn.h>
#include "adas/thneed/CL/cl.h"

#ifndef USE_PRECOMPILED

map<pair<cl_kernel, int>, string> g_args;
map<pair<cl_kernel, int>, int> g_args_size;
map<cl_program, string> g_program_source;

Thneed* g_thneed = NULL;
int g_fd = -1;

// Define function pointer types
typedef cl_program (*clCreateProgramWithSource_t)(cl_context, cl_uint, const char**, const size_t*, cl_int*);
typedef cl_int (*clBuildProgram_t)(cl_program, cl_uint, const cl_device_id*, const char*,
                                   void (*pfn_notify)(cl_program, void*), void*);
typedef cl_int (*clGetProgramInfo_t)(cl_program, cl_program_info, size_t, void*, size_t*);
typedef cl_program (*clCreateProgramWithBinary_t)(cl_context, cl_uint, const cl_device_id*, const size_t*,
                                                  const unsigned char**, cl_int*, cl_int*);
typedef cl_int (*clGetPlatformIDs_t)(cl_uint, cl_platform_id*, cl_uint*);
typedef cl_int (*clGetDeviceIDs_t)(cl_platform_id, cl_device_type, cl_uint, cl_device_id*, cl_uint*);
typedef cl_mem (*clCreateBuffer_t)(cl_context, cl_mem_flags, size_t, void*, cl_int*);
typedef cl_mem (*clCreateImage_t)(cl_context, cl_mem_flags, const cl_image_format*, const cl_image_desc*, void*,
                                  cl_int*);
typedef void* (*clEnqueueMapBuffer_t)(cl_command_queue, cl_mem, cl_bool, cl_map_flags, size_t, size_t, cl_uint,
                                      const cl_event*, cl_event*, cl_int*);
typedef cl_int (*clFinish_t)(cl_command_queue);
typedef cl_int (*clGetMemObjectInfo_t)(cl_mem, cl_mem_info, size_t, void*, size_t*);
typedef cl_int (*clGetImageInfo_t)(cl_mem, cl_image_info, size_t, void*, size_t*);
typedef cl_context (*clCreateContext_t)(const cl_context_properties*, cl_uint, const cl_device_id*,
                                        void(CL_CALLBACK*)(const char*, const void*, size_t, void*), void*, cl_int*);
typedef cl_command_queue (*clCreateCommandQueueWithProperties_t)(cl_context, cl_device_id, const cl_queue_properties*,
                                                                 cl_int*);
typedef cl_int (*clEnqueueWriteBuffer_t)(cl_command_queue, cl_mem, cl_bool, size_t, size_t, const void*, cl_uint,
                                         const cl_event*, cl_event*);
typedef cl_int (*clEnqueueReadBuffer_t)(cl_command_queue, cl_mem, cl_bool, size_t, size_t, void*, cl_uint,
                                        const cl_event*, cl_event*);
typedef cl_int (*clReleaseMemObject_t)(cl_mem);

// and log which one worked.
static void* load_opencl()
{
  static const char* candidates[] = {
      "libOpenCL.so",
      "/vendor/lib64/libOpenCL.so",
      "/system/vendor/lib64/libOpenCL.so",
      "libOpenCL.so.1",
  };
  for (const char* name : candidates) {
    dlerror();
    void* h = dlopen(name, RTLD_LAZY | RTLD_LOCAL);
    if (h != NULL) {
      __android_log_print(ANDROID_LOG_INFO, "thneed", "OpenCL loaded from %s", name);
      return h;
    }
    const char* err = dlerror();
    __android_log_print(ANDROID_LOG_WARN, "thneed", "dlopen(%s): %s", name, err ? err : "(null)");
  }
  return NULL;
}

void* opencl_library = load_opencl();

auto p_clCreateProgramWithSource = reinterpret_cast<clCreateProgramWithSource_t>(dlsym(opencl_library, "clCreateProgram"
                                                                                                       "WithSource"));
auto p_clBuildProgram = reinterpret_cast<clBuildProgram_t>(dlsym(opencl_library, "clBuildProgram"));
auto p_clGetProgramInfo = reinterpret_cast<clGetProgramInfo_t>(dlsym(opencl_library, "clGetProgramInfo"));
auto p_clCreateProgramWithBinary = reinterpret_cast<clCreateProgramWithBinary_t>(dlsym(opencl_library, "clCreateProgram"
                                                                                                       "WithBinary"));
auto p_clGetPlatformIDs = reinterpret_cast<clGetPlatformIDs_t>(dlsym(opencl_library, "clGetPlatformIDs"));
auto p_clGetDeviceIDs = reinterpret_cast<clGetDeviceIDs_t>(dlsym(opencl_library, "clGetDeviceIDs"));
auto p_clCreateBuffer = reinterpret_cast<clCreateBuffer_t>(dlsym(opencl_library, "clCreateBuffer"));
auto p_clCreateImage = reinterpret_cast<clCreateImage_t>(dlsym(opencl_library, "clCreateImage"));
auto p_clEnqueueMapBuffer = reinterpret_cast<clEnqueueMapBuffer_t>(dlsym(opencl_library, "clEnqueueMapBuffer"));
auto p_clFinish = reinterpret_cast<clFinish_t>(dlsym(opencl_library, "clFinish"));
auto p_clGetMemObjectInfo = reinterpret_cast<clGetMemObjectInfo_t>(dlsym(opencl_library, "clGetMemObjectInfo"));
auto p_clGetImageInfo = reinterpret_cast<clGetImageInfo_t>(dlsym(opencl_library, "clGetImageInfo"));
auto p_clCreateContext = reinterpret_cast<clCreateContext_t>(dlsym(opencl_library, "clCreateContext"));
auto p_clCreateCommandQueueWithProperties =
    reinterpret_cast<clCreateCommandQueueWithProperties_t>(dlsym(opencl_library, "clCreateCommandQueueWithProperties"));
auto p_clEnqueueWriteBuffer = reinterpret_cast<clEnqueueWriteBuffer_t>(dlsym(opencl_library, "clEnqueueWriteBuffer"));
auto p_clEnqueueReadBuffer = reinterpret_cast<clEnqueueReadBuffer_t>(dlsym(opencl_library, "clEnqueueReadBuffer"));
auto p_clReleaseMemObject = reinterpret_cast<clReleaseMemObject_t>(dlsym(opencl_library, "clReleaseMemObject"));

// Define more function pointer types
typedef cl_kernel (*clCreateKernel_t)(cl_program, const char*, cl_int*);
typedef cl_int (*clGetKernelArgInfo_t)(cl_kernel, cl_uint, cl_kernel_arg_info, size_t, void*, size_t*);
typedef cl_int (*clEnqueueNDRangeKernel_t)(cl_command_queue, cl_kernel, cl_uint, const size_t*, const size_t*,
                                           const size_t*, cl_uint, const cl_event*, cl_event*);
typedef cl_int (*clGetKernelInfo_t)(cl_kernel, cl_kernel_info, size_t, void*, size_t*);
typedef cl_int (*clSetKernelArg_t)(cl_kernel, cl_uint, size_t, const void*);

auto p_clCreateKernel = reinterpret_cast<clCreateKernel_t>(dlsym(opencl_library, "clCreateKernel"));
auto p_clGetKernelArgInfo = reinterpret_cast<clGetKernelArgInfo_t>(dlsym(opencl_library, "clGetKernelArgInfo"));
auto p_clEnqueueNDRangeKernel = reinterpret_cast<clEnqueueNDRangeKernel_t>(dlsym(opencl_library, "clEnqueueNDRangeKerne"
                                                                                                 "l"));
typedef cl_int (*clGetDeviceInfo_t)(cl_device_id, cl_device_info, size_t, void*, size_t*);
auto p_clGetDeviceInfo = reinterpret_cast<clGetDeviceInfo_t>(dlsym(opencl_library, "clGetDeviceInfo"));

auto p_clGetKernelInfo = reinterpret_cast<clGetKernelInfo_t>(dlsym(opencl_library, "clGetKernelInfo"));
auto p_clSetKernelArg = reinterpret_cast<clSetKernelArg_t>(dlsym(opencl_library, "clSetKernelArg"));

#undef assert
#define assert(x)                                                                                                      \
  ((x) ? __assert_no_op : (void)__android_log_print(ANDROID_LOG_ERROR, "ASSERT", "Assert failed: %s", #x))

void hexdump(uint8_t* d, int len)
{
  assert((len % 4) == 0);
  __android_log_print(ANDROID_LOG_INFO, "JNILOG", "  dumping %p len 0x%x\n", d, len);
  for (int i = 0; i < len / 4; i++) {
    if (i != 0 && (i % 0x10) == 0)
      __android_log_print(ANDROID_LOG_INFO, "JNILOG", "\n");
    __android_log_print(ANDROID_LOG_INFO, "JNILOG", "%8x ", d[i]);
  }
  __android_log_print(ANDROID_LOG_INFO, "JNILOG", "\n");
}

extern map<cl_program, string> g_program_source;

template <typename Func, typename Id, typename Name>
std::string get_info(Func get_info_func, Id id, Name param_name)
{
  size_t size = 0;
  CL_CHECK(get_info_func(id, param_name, 0, NULL, &size));
  std::string info(size, '\0');
  CL_CHECK(get_info_func(id, param_name, size, info.data(), NULL));
  return info;
}
inline std::string get_platform_info(cl_platform_id id, cl_platform_info name)
{
  return get_info(&clGetPlatformInfo, id, name);
}

cl_program cl_program_from_source(cl_context ctx, cl_device_id device_id, const std::string& src, const char* args)
{
  const char* csrc = src.c_str();
  cl_program prg = CL_CHECK_ERR((*p_clCreateProgramWithSource)(ctx, 1, &csrc, NULL, &err));
  if (int err = (*p_clBuildProgram)(prg, 1, &device_id, args, NULL, NULL); err != 0) {
    assert(0);
  }
  // Keep the source: it is needed if this run is later saved, and a built program can no longer be
  // asked for it — `clGetProgramInfo` returns a binary, not text. This line went missing during the
  // port, and `save(binaries = false)` silently ran into an empty table.
  g_program_source[prg] = src;
  return prg;
}

cl_program cl_program_from_binary(cl_context ctx, cl_device_id device_id, const uint8_t* binary, size_t length,
                                  const char* args)
{
  cl_program prg = CL_CHECK_ERR((*p_clCreateProgramWithBinary)(ctx, 1, &device_id, &length, &binary, NULL, &err));
  if (int err = (*p_clBuildProgram)(prg, 1, &device_id, args, NULL, NULL); err != 0) {
    assert(0);
  }
  return prg;
}

cl_device_id cl_get_device_id(cl_device_type device_type)
{
  cl_uint num_platforms = 0;
  CL_CHECK((*p_clGetPlatformIDs)(0, NULL, &num_platforms));
  std::unique_ptr<cl_platform_id[]> platform_ids = std::make_unique<cl_platform_id[]>(num_platforms);
  CL_CHECK((*p_clGetPlatformIDs)(num_platforms, &platform_ids[0], NULL));

  for (size_t i = 0; i < num_platforms; ++i) {
    if (cl_device_id device_id = NULL;
        (*p_clGetDeviceIDs)(platform_ids[i], device_type, 1, &device_id, NULL) == 0 && device_id) {
      return device_id;
    }
  }
  assert(0);
  return nullptr;
}

#include <dirent.h>
#include <unistd.h>
#include <iostream>

cl_int thneed_clSetKernelArg(cl_kernel kernel, cl_uint arg_index, size_t arg_size, const void* arg_value)
{
  g_args_size[make_pair(kernel, arg_index)] = arg_size;
  if (arg_value != NULL) {
    g_args[make_pair(kernel, arg_index)] = string((char*)arg_value, arg_size);
  } else {
    g_args[make_pair(kernel, arg_index)] = string("");
  }
  cl_int ret = (*p_clSetKernelArg)(kernel, arg_index, arg_size, arg_value);
  return ret;
}

void getGPUMemoryAllocationFD()
{
  DIR* dir = opendir("/proc/self/fd");
  struct dirent* entry;
  while ((entry = readdir(dir)) != NULL) {
    std::string fdPath = std::string("/proc/self/fd/") + entry->d_name;
    char linkTarget[256];
    ssize_t len = readlink(fdPath.c_str(), linkTarget, sizeof(linkTarget) - 1);
    if (len != -1) {
      linkTarget[len] = '\0';
      if (std::string(linkTarget) == "/dev/kgsl-3d0") {
        g_fd = std::stoi(entry->d_name);
        __android_log_print(ANDROID_LOG_INFO, "JNILOG", "File descriptor found for GPU allocation: %d", g_fd);
        closedir(dir);
        return;
      }
    }
  }

  // hmm, didn't find anything...
  closedir(dir);
}

std::string readFileIntoString(const char* filepath)
{
  std::ifstream ifs(filepath);
  std::stringstream buffer;
  buffer << ifs.rdbuf();
  return buffer.str();
}

void Thneed::load(uint8_t* buf)
{
  __android_log_print(ANDROID_LOG_INFO, "JNILOG", "Thneed::load: loading from byte array\n");

  int jsz = *(int*)buf;
  string jsonerr;
  string jj((char*)buf + sizeof(int), jsz);
  json11::Json jdat = json11::Json::parse(jj, jsonerr);

  map<cl_mem, cl_mem> real_mem;
  real_mem[NULL] = NULL;

  int ptr = sizeof(int) + jsz;
  for (auto& obj : jdat["objects"].array_items()) {
    auto mobj = obj.object_items();
    int sz = mobj["size"].int_value();
    cl_mem clbuf = NULL;

    if (mobj["buffer_id"].string_value().size() > 0) {
      clbuf = real_mem[*(cl_mem*)(mobj["buffer_id"].string_value().data())];
      assert(mobj["needs_load"].bool_value() == false);
    } else {
      if (mobj["needs_load"].bool_value()) {
        clbuf = (*p_clCreateBuffer)(context, CL_MEM_COPY_HOST_PTR | CL_MEM_READ_WRITE, sz, &buf[ptr], NULL);
        // Keep our own copy: it is needed if this run is later saved.
        loaded_constants[clbuf] = std::string((const char*)&buf[ptr], sz);
        if (debug >= 1)
          __android_log_print(ANDROID_LOG_INFO, "JNILOG", "loading %p %d @ 0x%X\n", clbuf, sz, ptr);
        ptr += sz;
      } else {
        void* host_zeros = calloc(sz, 1);
        clbuf = (*p_clCreateBuffer)(context, CL_MEM_COPY_HOST_PTR | CL_MEM_READ_WRITE, sz, host_zeros, NULL);
        free(host_zeros);
      }
    }
    assert(clbuf != NULL);

    if (mobj["arg_type"] == "image2d_t" || mobj["arg_type"] == "image1d_t") {
      cl_image_desc desc = {0};
      desc.image_type = (mobj["arg_type"] == "image2d_t") ? CL_MEM_OBJECT_IMAGE2D : CL_MEM_OBJECT_IMAGE1D_BUFFER;
      desc.image_width = mobj["width"].int_value();
      desc.image_height = mobj["height"].int_value();
      desc.image_row_pitch = mobj["row_pitch"].int_value();
      assert(sz == desc.image_height * desc.image_row_pitch);
      desc.buffer = clbuf;
      cl_image_format format = {0};
      format.image_channel_order = CL_RGBA;
      format.image_channel_data_type = mobj["float32"].bool_value() ? CL_FLOAT : CL_HALF_FLOAT;

      cl_int errcode;

      clbuf = (*p_clCreateImage)(context, CL_MEM_READ_WRITE, &format, &desc, NULL, &errcode);

      if (clbuf == NULL) {
        __android_log_print(ANDROID_LOG_INFO, "JNILOG", "clError: %d create image %zux%zu rp %zu with buffer %p\n",
                            errcode, desc.image_width, desc.image_height, desc.image_row_pitch, desc.buffer);
      }
      assert(clbuf != NULL);
    }

    real_mem[*(cl_mem*)(mobj["id"].string_value().data())] = clbuf;
  }

  map<string, cl_program> g_programs;
  for (const auto& [name, source] : jdat["programs"].object_items()) {
    if (debug >= 1)
      __android_log_print(ANDROID_LOG_INFO, "JNILOG", "building %s with size %zu\n", name.c_str(),
                          source.string_value().size());
    g_programs[name] = cl_program_from_source(context, device_id, source.string_value());
  }

  for (auto& obj : jdat["inputs"].array_items()) {
    auto mobj = obj.object_items();
    int sz = mobj["size"].int_value();
    cl_mem aa = real_mem[*(cl_mem*)(mobj["buffer_id"].string_value().data())];
    input_clmem.push_back(aa);
    input_sizes.push_back(sz);
    input_names.push_back(mobj["name"].string_value());
    __android_log_print(ANDROID_LOG_INFO, "JNILOG", "Thneed::load: adding input %s with size %d\n",
                        mobj["name"].string_value().data(), sz);

    cl_int cl_err;
    void* ret = (*p_clEnqueueMapBuffer)(command_queue, aa, CL_TRUE, CL_MAP_WRITE, 0, sz, 0, NULL, NULL, &cl_err);
    // cl_get_error_string(cl_err), aa, sz);
    assert(cl_err == CL_SUCCESS);
    inputs.push_back(ret);
  }

  for (auto& obj : jdat["outputs"].array_items()) {
    auto mobj = obj.object_items();
    int sz = mobj["size"].int_value();
    __android_log_print(ANDROID_LOG_INFO, "JNILOG", "Thneed::save: adding output with size %d\n", sz);
    output = real_mem[*(cl_mem*)(mobj["buffer_id"].string_value().data())];
    output_size = sz;
    if (output == NULL)
      __android_log_print(ANDROID_LOG_INFO, "JNILOG", "Thneed::save: output was null!");
  }

  for (auto& obj : jdat["binaries"].array_items()) {
    string name = obj["name"].string_value();
    size_t length = obj["length"].int_value();
    if (debug >= 1)
      __android_log_print(ANDROID_LOG_INFO, "JNILOG", "binary %s with size %zu\n", name.c_str(), length);
    g_programs[name] = cl_program_from_binary(context, device_id, (const uint8_t*)&buf[ptr], length);
    ptr += length;
  }

  for (auto& obj : jdat["kernels"].array_items()) {
    auto gws = obj["global_work_size"];
    auto lws = obj["local_work_size"];
    auto kk = shared_ptr<CLQueuedKernel>(new CLQueuedKernel(this));

    kk->name = obj["name"].string_value();
    kk->program = g_programs[kk->name];
    kk->work_dim = obj["work_dim"].int_value();
    for (int i = 0; i < kk->work_dim; i++) {
      kk->global_work_size[i] = gws[i].int_value();
      kk->local_work_size[i] = lws[i].int_value();
    }
    kk->num_args = obj["num_args"].int_value();
    for (int i = 0; i < kk->num_args; i++) {
      string arg = obj["args"].array_items()[i].string_value();
      int arg_size = obj["args_size"].array_items()[i].int_value();
      kk->args_size.push_back(arg_size);
      if (arg_size == 8) {
        cl_mem val = *(cl_mem*)(arg.data());
        val = real_mem[val];
        kk->args.push_back(string((char*)&val, sizeof(val)));
      } else {
        kk->args.push_back(arg);
      }
    }
    kq.push_back(kk);
  }

  (*p_clFinish)(command_queue);
}

#ifndef QCOM2

Thneed::Thneed(bool do_clinit, cl_context _context)
{
  context = _context;
  if (do_clinit)
    clinit();
  debug = 0;
}

void Thneed::execute(float** finputs, float* foutput, bool slow)
{
  uint64_t tb, te;
  if (debug >= 1)
    tb = nanos_since_boot();

  // ****** copy inputs
  copy_inputs(finputs);

  // ****** run commands
  clexec();

  // ****** copy outputs
  copy_output(foutput);

  if (debug >= 1) {
    te = nanos_since_boot();
    __android_log_print(ANDROID_LOG_INFO, "JNILOG", "model exec in %lu us\n", (te - tb) / 1000);
  }
}

#else

int __ioctl(int filedes, unsigned long request, void* argp)
{
  request &= 0xFFFFFFFF;
  Thneed* thneed = g_thneed;

  if (request == IOCTL_KGSL_DRAWCTXT_CREATE) {
    struct kgsl_drawctxt_create* create = (struct kgsl_drawctxt_create*)argp;
    create->flags &= ~KGSL_CONTEXT_PRIORITY_MASK;
    create->flags |= 6 << KGSL_CONTEXT_PRIORITY_SHIFT;
    __android_log_print(ANDROID_LOG_INFO, "JNILOG", "IOCTL_KGSL_DRAWCTXT_CREATE: creating context with flags 0x%x\n",
                        create->flags);
  }

  if (thneed != NULL) {
    if (request == IOCTL_KGSL_GPU_COMMAND) {
      struct kgsl_gpu_command* cmd = (struct kgsl_gpu_command*)argp;
      if (thneed->record) {
        thneed->timestamp = cmd->timestamp;
        thneed->context_id = cmd->context_id;
        thneed->cmds.push_back(unique_ptr<CachedCommand>(new CachedCommand(thneed, cmd)));
      }
      if (thneed->debug >= 1) {
        __android_log_print(ANDROID_LOG_INFO, "JNILOG",
                            "IOCTL_KGSL_GPU_COMMAND(%2zu): flags: 0x%lx    context_id: %u  timestamp: %u  numcmds: %d  "
                            "numobjs: %d\n",
                            thneed->cmds.size(), cmd->flags, cmd->context_id, cmd->timestamp, cmd->numcmds,
                            cmd->numobjs);
      }
    } else if (request == IOCTL_KGSL_GPUOBJ_SYNC) {
      struct kgsl_gpuobj_sync* cmd = (struct kgsl_gpuobj_sync*)argp;
      struct kgsl_gpuobj_sync_obj* objs = (struct kgsl_gpuobj_sync_obj*)(cmd->objs);

      if (thneed->debug >= 2) {
        __android_log_print(ANDROID_LOG_INFO, "JNILOG", "IOCTL_KGSL_GPUOBJ_SYNC count:%d ", cmd->count);
        for (int i = 0; i < cmd->count; i++) {
          __android_log_print(ANDROID_LOG_INFO, "JNILOG", " -- offset:0x%lx len:0x%lx id:%d op:%d  ", objs[i].offset,
                              objs[i].length, objs[i].id, objs[i].op);
        }
        __android_log_print(ANDROID_LOG_INFO, "JNILOG", "\n");
      }

      if (thneed->record) {
        thneed->cmds.push_back(unique_ptr<CachedSync>(
            new CachedSync(thneed, string((char*)objs, sizeof(struct kgsl_gpuobj_sync_obj) * cmd->count))));
      }
    } else if (request == IOCTL_KGSL_DEVICE_WAITTIMESTAMP_CTXTID) {
      struct kgsl_device_waittimestamp_ctxtid* cmd = (struct kgsl_device_waittimestamp_ctxtid*)argp;
      if (thneed->debug >= 1) {
        __android_log_print(ANDROID_LOG_INFO, "JNILOG",
                            "IOCTL_KGSL_DEVICE_WAITTIMESTAMP_CTXTID: context_id: %d  timestamp: %d  timeout: %d\n",
                            cmd->context_id, cmd->timestamp, cmd->timeout);
      }
    } else if (request == IOCTL_KGSL_SETPROPERTY) {
      if (thneed->debug >= 1) {
        struct kgsl_device_getproperty* prop = (struct kgsl_device_getproperty*)argp;
        __android_log_print(ANDROID_LOG_INFO, "JNILOG", "IOCTL_KGSL_SETPROPERTY: 0x%x sizebytes:%zu\n", prop->type,
                            prop->sizebytes);
        if (thneed->debug >= 2) {
          hexdump((uint8_t*)prop->value, prop->sizebytes);
          if (prop->type == KGSL_PROP_PWR_CONSTRAINT) {
            struct kgsl_device_constraint* constraint = (struct kgsl_device_constraint*)prop->value;
            hexdump((uint8_t*)constraint->data, constraint->size);
          }
        }
      }
    } else if (request == IOCTL_KGSL_DRAWCTXT_CREATE || request == IOCTL_KGSL_DRAWCTXT_DESTROY) {
    } else if (request == IOCTL_KGSL_GPUOBJ_ALLOC || request == IOCTL_KGSL_GPUOBJ_FREE) {
    } else {
      if (thneed->debug >= 1) {
        __android_log_print(ANDROID_LOG_INFO, "JNILOG", "other ioctl %lx\n", request);
      }
    }
  }

  int ret = ioctl(filedes, request, argp);
  if (ret != 0)
    __android_log_print(ANDROID_LOG_INFO, "JNILOG", "ioctl returned %d with errno %d\n", ret, errno);
  return ret;
}

// *********** GPUMalloc ***********

GPUMalloc::GPUMalloc(int size, int fd)
{
  struct kgsl_gpuobj_alloc alloc;
  memset(&alloc, 0, sizeof(alloc));
  alloc.size = size;
  alloc.flags = 0x10000a00;
  __ioctl(fd, IOCTL_KGSL_GPUOBJ_ALLOC, &alloc);
  void* addr = mmap64(NULL, alloc.mmapsize, 0x3, 0x1, fd, alloc.id * 0x1000);
  assert(addr != MAP_FAILED);

  base = (uint64_t)addr;
  remaining = size;
}

GPUMalloc::~GPUMalloc() {}

void* GPUMalloc::alloc(int size)
{
  void* ret = (void*)base;
  size = (size + 0xff) & (~0xFF);
  assert(size <= remaining);
  remaining -= size;
  base += size;
  return ret;
}

// *********** CachedSync, at the ioctl layer ***********

void CachedSync::exec()
{
  struct kgsl_gpuobj_sync cmd;

  cmd.objs = (uint64_t)data.data();
  cmd.obj_len = data.length();
  cmd.count = data.length() / sizeof(struct kgsl_gpuobj_sync_obj);

  int ret = __ioctl(thneed->fd, IOCTL_KGSL_GPUOBJ_SYNC, &cmd);
  assert(ret == 0);
}

// *********** CachedCommand, at the ioctl layer ***********

CachedCommand::CachedCommand(Thneed* lthneed, struct kgsl_gpu_command* cmd)
{
  thneed = lthneed;
  assert(cmd->numsyncs == 0);

  memcpy(&cache, cmd, sizeof(cache));

  if (cmd->numcmds > 0) {
    cmds = make_unique<struct kgsl_command_object[]>(cmd->numcmds);
    memcpy(cmds.get(), (void*)cmd->cmdlist, sizeof(struct kgsl_command_object) * cmd->numcmds);
    cache.cmdlist = (uint64_t)cmds.get();
    for (int i = 0; i < cmd->numcmds; i++) {
      void* nn = thneed->ram->alloc(cmds[i].size);
      memcpy(nn, (void*)cmds[i].gpuaddr, cmds[i].size);
      cmds[i].gpuaddr = (uint64_t)nn;
    }
  }

  if (cmd->numobjs > 0) {
    objs = make_unique<struct kgsl_command_object[]>(cmd->numobjs);
    memcpy(objs.get(), (void*)cmd->objlist, sizeof(struct kgsl_command_object) * cmd->numobjs);
    cache.objlist = (uint64_t)objs.get();
    for (int i = 0; i < cmd->numobjs; i++) {
      void* nn = thneed->ram->alloc(objs[i].size);
      memset(nn, 0, objs[i].size);
      objs[i].gpuaddr = (uint64_t)nn;
    }
  }

  kq = thneed->ckq;
  thneed->ckq.clear();
}

void CachedCommand::exec()
{
  cache.timestamp = ++thneed->timestamp;
  int ret = __ioctl(thneed->fd, IOCTL_KGSL_GPU_COMMAND, &cache);

  if (thneed->debug >= 1)
    __android_log_print(ANDROID_LOG_INFO, "JNILOG", "CachedCommand::exec got %d\n", ret);

  if (thneed->debug >= 2) {
    for (auto& it : kq) {
      it->debug_print(false);
    }
  }

  assert(ret == 0);
}

void Thneed::wait()
{
  struct kgsl_device_waittimestamp_ctxtid wait;
  wait.context_id = context_id;
  wait.timestamp = timestamp;
  wait.timeout = -1;

  uint64_t tb = nanos_since_boot();
  int wret = __ioctl(fd, IOCTL_KGSL_DEVICE_WAITTIMESTAMP_CTXTID, &wait);
  uint64_t te = nanos_since_boot();

  if (debug >= 1)
    __android_log_print(ANDROID_LOG_INFO, "JNILOG", "wait %d after %lu us\n", wret, (te - tb) / 1000);
}

Thneed::Thneed(bool do_clinit, cl_context _context)
{
  if (do_clinit)
    clinit();
  getGPUMemoryAllocationFD();
  assert(g_fd != -1);
  fd = g_fd;
  ram = make_unique<GPUMalloc>(0x80000, fd);
  timestamp = -1;
  g_thneed = this;
  debug = 1;
}

void Thneed::execute(float** finputs, float* foutput, bool slow)
{
  uint64_t tb, te;
  if (debug >= 1)
    tb = nanos_since_boot();

  // ****** copy inputs
  copy_inputs(finputs, true);

  int i = 0;
  for (auto& it : cmds) {
    ++i;
    if (debug >= 1)
      __android_log_print(ANDROID_LOG_INFO, "JNILOG", "run %2d @ %7lu us: ", i, (nanos_since_boot() - tb) / 1000);
    it->exec();
    if ((i == cmds.size()) || slow)
      wait();
  }

  // ****** copy outputs
  copy_output(foutput);

  if (debug >= 1) {
    te = nanos_since_boot();
    __android_log_print(ANDROID_LOG_INFO, "JNILOG", "model exec in %lu us\n", (te - tb) / 1000);
  }
}

#endif

json11::Json CLQueuedKernel::to_json() const
{
  return json11::Json::object{
      {"name", name},
      {"work_dim", (int)work_dim},
      {"global_work_size",
       json11::Json::array{(int)global_work_size[0], (int)global_work_size[1], (int)global_work_size[2]}},
      {"local_work_size",
       json11::Json::array{(int)local_work_size[0], (int)local_work_size[1], (int)local_work_size[2]}},
      {"num_args", (int)num_args},
      {"args", args},
      {"args_size", args_size},
  };
}

bool Thneed::save(const char* filename, bool save_binaries)
{
  __android_log_print(ANDROID_LOG_INFO, "JNILOG", "Thneed::save: %s (%s)\n", filename,
                      save_binaries ? "binaries" : "sources");

  std::vector<json11::Json> kernels;
  std::set<std::string> saved_objects;
  std::vector<json11::Json> objects;
  std::map<std::string, std::string> programs;
  std::map<std::string, std::string> binaries;

  // Which buffers carry weights. Upstream recognised them by the argument names `weights` and
  // `biases`, but only the generator of that era named them so; tinygrad calls everything `data0_32`,
  // and matching by name finds nothing — the file was written without a single weight and looked
  // healthy. A structural test is sounder: a buffer read before anything was written to it cannot
  // have come from inside the graph. Argument zero of a kernel is its destination.
  std::set<std::string> written;
  std::set<std::string> constants;
  for (auto& k : kq) {
    for (size_t i = 0; i < k->args.size(); i++) {
      if (k->args[i].size() != 8)
        continue;
      if (i == 0)
        written.insert(k->args[i]);
      else if (written.find(k->args[i]) == written.end())
        constants.insert(k->args[i]);
    }
  }

  for (auto& k : kq) {
    kernels.push_back(k->to_json());

    // Pointer-sized arguments are buffers; describe each one once.
    int i = 0;
    for (auto& a : k->args) {
      if (i >= (int)k->arg_types.size()) {
        // Argument types are filled in on the kernel's first launch. Getting here earlier leaves
        // nothing to write: the buffer type is unknown, and guessing it means writing a file that
        // will not load.
        __android_log_print(ANDROID_LOG_ERROR, "JNILOG", "Thneed::save: %s has never been launched\n", k->name.c_str());
        return false;
      }
      if (a.size() == 8 && saved_objects.find(a) == saved_objects.end()) {
        saved_objects.insert(a);
        cl_mem val = *(cl_mem*)(a.data());
        if (val != NULL) {
          const bool needs_load = constants.find(a) != constants.end();
          auto jj = json11::Json::object({{"id", a}, {"arg_type", k->arg_types[i]}});

          if (k->arg_types[i] == "image2d_t" || k->arg_types[i] == "image1d_t") {
            cl_mem buf = NULL;
            (*p_clGetImageInfo)(val, CL_IMAGE_BUFFER, sizeof(buf), &buf, NULL);
            std::string aa((char*)&buf, sizeof(buf));
            jj["buffer_id"] = aa;

            size_t width = 0, height = 0, row_pitch = 0;
            (*p_clGetImageInfo)(val, CL_IMAGE_WIDTH, sizeof(width), &width, NULL);
            (*p_clGetImageInfo)(val, CL_IMAGE_HEIGHT, sizeof(height), &height, NULL);
            (*p_clGetImageInfo)(val, CL_IMAGE_ROW_PITCH, sizeof(row_pitch), &row_pitch, NULL);
            jj["width"] = (int)width;
            jj["height"] = (int)height;
            jj["row_pitch"] = (int)row_pitch;
            jj["size"] = (int)(height * row_pitch);
            jj["needs_load"] = false;

            if (saved_objects.find(aa) == saved_objects.end()) {
              saved_objects.insert(aa);
              size_t sz = 0;
              (*p_clGetMemObjectInfo)(buf, CL_MEM_SIZE, sizeof(sz), &sz, NULL);
              objects.push_back(json11::Json::object(
                  {{"id", aa}, {"arg_type", "<image buffer>"}, {"needs_load", needs_load}, {"size", (int)sz}}));
            }
          } else {
            size_t sz = 0;
            (*p_clGetMemObjectInfo)(val, CL_MEM_SIZE, sizeof(sz), &sz, NULL);
            jj["size"] = (int)sz;
            jj["needs_load"] = needs_load;
          }
          objects.push_back(jj);
        }
      }
      i++;
    }

    if (save_binaries) {
      size_t binary_size = 0;
      if ((*p_clGetProgramInfo)(k->program, CL_PROGRAM_BINARY_SIZES, sizeof(binary_size), &binary_size, NULL) != 0 ||
          binary_size == 0) {
        __android_log_print(ANDROID_LOG_ERROR, "JNILOG", "Thneed::save: no binary for %s\n", k->name.c_str());
        return false;
      }
      std::string sv(binary_size, '\x00');
      uint8_t* bufs[1] = {(uint8_t*)sv.data()};
      if ((*p_clGetProgramInfo)(k->program, CL_PROGRAM_BINARIES, sizeof(bufs), &bufs, NULL) != 0) {
        __android_log_print(ANDROID_LOG_ERROR, "JNILOG", "Thneed::save: cannot read binary for %s\n", k->name.c_str());
        return false;
      }
      binaries[k->name] = sv;
    } else {
      const auto it = g_program_source.find(k->program);
      if (it == g_program_source.end()) {
        // The program came from a binary — it has no source and there is nowhere to get one.
        // Silently writing a file without the kernel means handing over a model known to be broken.
        __android_log_print(ANDROID_LOG_ERROR, "JNILOG",
                            "Thneed::save: no source for %s — the thneed was loaded from binaries\n", k->name.c_str());
        return false;
      }
      programs[k->name] = it->second;
    }
  }

  std::vector<std::string> saved_buffers;
  for (auto& obj : objects) {
    auto mobj = obj.object_items();
    if (!mobj["needs_load"].bool_value())
      continue;
    cl_mem val = *(cl_mem*)(mobj["id"].string_value().data());
    const int sz = mobj["size"].int_value();
    if (mobj["arg_type"] == "image2d_t" || mobj["arg_type"] == "image1d_t") {
      __android_log_print(ANDROID_LOG_ERROR, "JNILOG", "Thneed::save: cannot read back an image\n");
      return false;
    }
    const auto known = loaded_constants.find(val);
    if (known != loaded_constants.end()) {
      if ((int)known->second.size() != sz) {
        __android_log_print(ANDROID_LOG_ERROR, "JNILOG", "Thneed::save: size drift %zu vs %d\n", known->second.size(),
                            sz);
        return false;
      }
      saved_buffers.push_back(known->second);
      continue;
    }

    std::string buf(sz, '\x00');
    if ((*p_clEnqueueReadBuffer)(command_queue, val, CL_TRUE, 0, sz, (void*)buf.data(), 0, NULL, NULL) != CL_SUCCESS) {
      __android_log_print(ANDROID_LOG_ERROR, "JNILOG", "Thneed::save: buffer read failed\n");
      return false;
    }
    saved_buffers.push_back(buf);
  }

  std::vector<json11::Json> jbinaries;
  for (auto& obj : binaries) {
    jbinaries.push_back(json11::Json::object({{"name", obj.first}, {"length", (int)obj.second.size()}}));
    saved_buffers.push_back(obj.second);
  }

  // Inputs and output. In 0.8.16, where this code comes from, these fields did not exist yet, and
  // without them the loader cannot find where to put the frame or where to take the answer from: the
  // model runs and returns zeros while looking healthy. The order must match the one seen at load
  // time — the binding is positional.
  std::vector<json11::Json> jinputs;
  for (size_t i = 0; i < input_clmem.size(); i++) {
    const std::string key((const char*)&input_clmem[i], sizeof(cl_mem));
    if (saved_objects.find(key) == saved_objects.end()) {
      // A buffer no kernel touches — a placeholder standing in for a missing input, say. It still
      // needs an entry in the object table, otherwise the positions shift.
      saved_objects.insert(key);
      objects.push_back(json11::Json::object(
          {{"id", key}, {"arg_type", "float*"}, {"size", (int)input_sizes[i]}, {"needs_load", false}}));
    }
    jinputs.push_back(json11::Json::object({{"name", i < input_names.size() ? input_names[i] : std::string()},
                                            {"size", (int)input_sizes[i]},
                                            {"buffer_id", key}}));
  }

  std::vector<json11::Json> joutputs;
  if (output != NULL && output_size > 0) {
    const std::string key((const char*)&output, sizeof(cl_mem));
    if (saved_objects.find(key) == saved_objects.end()) {
      // Same as for the inputs: without an object in the table the reference dangles, the loader
      // gets NULL and logs "output was null", and the model returns zeros while looking healthy.
      saved_objects.insert(key);
      objects.push_back(
          json11::Json::object({{"id", key}, {"arg_type", "float*"}, {"size", output_size}, {"needs_load", false}}));
    }
    joutputs.push_back(json11::Json::object({{"size", output_size}, {"buffer_id", key}}));
  }

  json11::Json jdat = json11::Json::object({
      {"kernels", kernels},
      {"objects", objects},
      {"programs", programs},
      {"binaries", jbinaries},
      {"inputs", jinputs},
      {"outputs", joutputs},
  });

  const std::string str = jdat.dump();
  const int jsz = (int)str.length();

  FILE* f = fopen(filename, "wb");
  if (f == NULL) {
    __android_log_print(ANDROID_LOG_ERROR, "JNILOG", "Thneed::save: cannot open %s\n", filename);
    return false;
  }
  bool ok = fwrite(&jsz, 1, sizeof(jsz), f) == sizeof(jsz);
  ok = ok && fwrite(str.data(), 1, jsz, f) == (size_t)jsz;
  for (auto& b : saved_buffers) {
    ok = ok && fwrite(b.data(), 1, b.length(), f) == b.length();
  }
  fclose(f);
  __android_log_print(ANDROID_LOG_INFO, "JNILOG", "Thneed::save: %d kernels, %zu blobs, %zu inputs, %s\n",
                      (int)kernels.size(), saved_buffers.size(), jinputs.size(), ok ? "written" : "TRUNCATED");
  return ok;
}

void Thneed::stop() { record = false; }

void Thneed::clinit()
{
  device_id = cl_get_device_id(CL_DEVICE_TYPE_DEFAULT);
  if (context == NULL)
    context = CL_CHECK_ERR((*p_clCreateContext)(NULL, 1, &device_id, NULL, NULL, &err));
  cl_command_queue_properties props[3] = {CL_QUEUE_PROPERTIES, 0, 0};
  command_queue = CL_CHECK_ERR((*p_clCreateCommandQueueWithProperties)(context, device_id, props, &err));
  __android_log_print(ANDROID_LOG_INFO, "JNILOG", "Thneed::clinit done\n");
}

cl_int Thneed::clexec()
{
  if (debug >= 1)
    __android_log_print(ANDROID_LOG_INFO, "JNILOG", "Thneed::clexec: running %lu queued kernels\n", kq.size());
  for (auto& k : kq) {
    if (record)
      ckq.push_back(k);
    cl_int ret = k->exec();
    assert(ret == CL_SUCCESS);
  }
  return (*p_clFinish)(command_queue);
}

void Thneed::copy_inputs(float** finputs, bool internal)
{
  for (int idx = 0; idx < inputs.size(); ++idx) {
    if (debug >= 1)
      __android_log_print(ANDROID_LOG_INFO, "JNILOG", "copying idx:%d %lu -- %p -> %p (cl %p)\n", idx, input_sizes[idx],
                          finputs[idx], inputs[idx], input_clmem[idx]);

    if (internal) {
      if (finputs[idx] != NULL)
        memcpy(inputs[idx], finputs[idx], input_sizes[idx]);
    } else {
      if (finputs[idx] != NULL)
        CL_CHECK((*p_clEnqueueWriteBuffer)(command_queue, input_clmem[idx], CL_TRUE, 0, input_sizes[idx], finputs[idx],
                                           0, NULL, NULL));
    }
  }
}

void Thneed::copy_output(float* foutput)
{
  if (output != NULL) {
    size_t sz;
    (*p_clGetMemObjectInfo)(output, CL_MEM_SIZE, sizeof(sz), &sz, NULL);
    if (debug >= 1)
      __android_log_print(ANDROID_LOG_INFO, "JNILOG", "copying %lu for output %p -> %p\n", sz, output, foutput);
    CL_CHECK((*p_clEnqueueReadBuffer)(command_queue, output, CL_TRUE, 0, sz, foutput, 0, NULL, NULL));
  } else {
    __android_log_print(ANDROID_LOG_INFO, "JNILOG", "CAUTION: model output is NULL, does it have no outputs?\n");
  }
}

// *********** CLQueuedKernel ***********

CLQueuedKernel::CLQueuedKernel(Thneed* lthneed, cl_kernel _kernel, cl_uint _work_dim, const size_t* _global_work_size,
                               const size_t* _local_work_size)
{
  thneed = lthneed;
  kernel = _kernel;
  work_dim = _work_dim;
  assert(work_dim <= 3);
  for (int i = 0; i < work_dim; i++) {
    global_work_size[i] = _global_work_size[i];
    local_work_size[i] = _local_work_size[i];
  }

  char _name[0x100];
  (*p_clGetKernelInfo)(kernel, CL_KERNEL_FUNCTION_NAME, sizeof(_name), _name, NULL);
  name = string(_name);
  (*p_clGetKernelInfo)(kernel, CL_KERNEL_NUM_ARGS, sizeof(num_args), &num_args, NULL);

  for (int i = 0; i < num_args; i++) {
    char arg_name[0x100] = {0};
    (*p_clGetKernelArgInfo)(kernel, i, CL_KERNEL_ARG_NAME, sizeof(arg_name), arg_name, NULL);
    arg_names.push_back(string(arg_name));
    (*p_clGetKernelArgInfo)(kernel, i, CL_KERNEL_ARG_TYPE_NAME, sizeof(arg_name), arg_name, NULL);
    arg_types.push_back(string(arg_name));

    args.push_back(g_args[make_pair(kernel, i)]);
    args_size.push_back(g_args_size[make_pair(kernel, i)]);
  }

  (*p_clGetKernelInfo)(kernel, CL_KERNEL_PROGRAM, sizeof(program), &program, NULL);
}

int CLQueuedKernel::get_arg_num(const char* search_arg_name)
{
  for (int i = 0; i < num_args; i++) {
    if (arg_names[i] == search_arg_name)
      return i;
  }
  __android_log_print(ANDROID_LOG_INFO, "JNILOG", "failed to find %s in %s\n", search_arg_name, name.c_str());
  assert(false);
  return -1;
}

cl_int CLQueuedKernel::exec()
{
  if (kernel == NULL) {
    kernel = (*p_clCreateKernel)(program, name.c_str(), NULL);
    arg_names.clear();
    arg_types.clear();

    for (int j = 0; j < num_args; j++) {
      char arg_name[0x100] = {0};
      (*p_clGetKernelArgInfo)(kernel, j, CL_KERNEL_ARG_NAME, sizeof(arg_name), arg_name, NULL);
      arg_names.push_back(string(arg_name));
      (*p_clGetKernelArgInfo)(kernel, j, CL_KERNEL_ARG_TYPE_NAME, sizeof(arg_name), arg_name, NULL);
      arg_types.push_back(string(arg_name));

      cl_int ret;
      if (args[j].size() != 0) {
        assert(args[j].size() == args_size[j]);
        ret = thneed_clSetKernelArg(kernel, j, args[j].size(), args[j].data());
      } else {
        ret = thneed_clSetKernelArg(kernel, j, args_size[j], NULL);
      }
      assert(ret == CL_SUCCESS);
    }
  }

  return (*p_clEnqueueNDRangeKernel)(thneed->command_queue, kernel, work_dim, NULL, global_work_size, local_work_size,
                                     0, NULL, NULL);
}

void CLQueuedKernel::debug_print(bool verbose)
{
  __android_log_print(ANDROID_LOG_INFO, "JNILOG", "%p %56s -- ", kernel, name.c_str());
  for (int i = 0; i < work_dim; i++) {
    __android_log_print(ANDROID_LOG_INFO, "JNILOG", "%4zu ", global_work_size[i]);
  }
  __android_log_print(ANDROID_LOG_INFO, "JNILOG", " -- ");
  for (int i = 0; i < work_dim; i++) {
    __android_log_print(ANDROID_LOG_INFO, "JNILOG", "%4zu ", local_work_size[i]);
  }
  __android_log_print(ANDROID_LOG_INFO, "JNILOG", "\n");

  if (verbose) {
    for (int i = 0; i < num_args; i++) {
      string arg = args[i];
      __android_log_print(ANDROID_LOG_INFO, "JNILOG", "  %s %s", arg_types[i].c_str(), arg_names[i].c_str());
      void* arg_value = (void*)arg.data();
      int arg_size = arg.size();
      if (arg_size == 0) {
        __android_log_print(ANDROID_LOG_INFO, "JNILOG", " (size) %d", args_size[i]);
      } else if (arg_size == 1) {
        __android_log_print(ANDROID_LOG_INFO, "JNILOG", " = %d", *((char*)arg_value));
      } else if (arg_size == 2) {
        __android_log_print(ANDROID_LOG_INFO, "JNILOG", " = %d", *((short*)arg_value));
      } else if (arg_size == 4) {
        if (arg_types[i] == "float") {
          __android_log_print(ANDROID_LOG_INFO, "JNILOG", " = %f", *((float*)arg_value));
        } else {
          __android_log_print(ANDROID_LOG_INFO, "JNILOG", " = %d", *((int*)arg_value));
        }
      } else if (arg_size == 8) {
        cl_mem val = (cl_mem)(*((uintptr_t*)arg_value));
        __android_log_print(ANDROID_LOG_INFO, "JNILOG", " = %p", val);
        if (val != NULL) {
          cl_mem_object_type obj_type;
          (*p_clGetMemObjectInfo)(val, CL_MEM_TYPE, sizeof(obj_type), &obj_type, NULL);
          if (arg_types[i] == "image2d_t" || arg_types[i] == "image1d_t" || obj_type == CL_MEM_OBJECT_IMAGE2D) {
            cl_image_format format;
            size_t width, height, depth, array_size, row_pitch, slice_pitch;
            cl_mem buf;
            (*p_clGetImageInfo)(val, CL_IMAGE_FORMAT, sizeof(format), &format, NULL);
            assert(format.image_channel_order == CL_RGBA);
            assert(format.image_channel_data_type == CL_HALF_FLOAT || format.image_channel_data_type == CL_FLOAT);
            (*p_clGetImageInfo)(val, CL_IMAGE_WIDTH, sizeof(width), &width, NULL);
            (*p_clGetImageInfo)(val, CL_IMAGE_HEIGHT, sizeof(height), &height, NULL);
            (*p_clGetImageInfo)(val, CL_IMAGE_ROW_PITCH, sizeof(row_pitch), &row_pitch, NULL);
            (*p_clGetImageInfo)(val, CL_IMAGE_DEPTH, sizeof(depth), &depth, NULL);
            (*p_clGetImageInfo)(val, CL_IMAGE_ARRAY_SIZE, sizeof(array_size), &array_size, NULL);
            (*p_clGetImageInfo)(val, CL_IMAGE_SLICE_PITCH, sizeof(slice_pitch), &slice_pitch, NULL);
            assert(depth == 0);
            assert(array_size == 0);
            assert(slice_pitch == 0);

            (*p_clGetImageInfo)(val, CL_IMAGE_BUFFER, sizeof(buf), &buf, NULL);
            size_t sz = 0;
            if (buf != NULL)
              (*p_clGetMemObjectInfo)(buf, CL_MEM_SIZE, sizeof(sz), &sz, NULL);
            __android_log_print(ANDROID_LOG_INFO, "JNILOG", " image %zu x %zu rp %zu @ %p buffer %zu", width, height,
                                row_pitch, buf, sz);
          } else {
            size_t sz;
            (*p_clGetMemObjectInfo)(val, CL_MEM_SIZE, sizeof(sz), &sz, NULL);
            __android_log_print(ANDROID_LOG_INFO, "JNILOG", " buffer %zu", sz);
          }
        }
      }
      __android_log_print(ANDROID_LOG_INFO, "JNILOG", "\n");
    }
  }
}

#endif

ThneedModel::ThneedModel(uint8_t* model, float* _output, size_t _output_size, int runtime, bool luse_tf8,
                         cl_context context)
{
  thneed = new Thneed(true, context);
  thneed->load(model);
  thneed->clexec();

  recorded = false;
  output = _output;
}

void* ThneedModel::getCLBuffer(const std::string name)
{
  int index = -1;
  for (int i = 0; i < inputs.size(); i++) {
    if (name == inputs[i]->name) {
      index = i;
      break;
    }
  }

  if (thneed->input_clmem.size() >= inputs.size()) {
    return &thneed->input_clmem[inputs.size() - index - 1];
  } else {
    return nullptr;
  }
}

void ThneedModel::execute()
{
  if (!recorded) {
    thneed->record = true;
    float* input_buffers[inputs.size()];
    for (int i = 0; i < inputs.size(); i++) {
      input_buffers[inputs.size() - i - 1] = inputs[i]->buffer;
    }

    thneed->copy_inputs(input_buffers);
    thneed->clexec();
    thneed->copy_output(output);
    thneed->stop();

    recorded = true;
  } else {
    float* input_buffers[inputs.size()];
    for (int i = 0; i < inputs.size(); i++) {
      input_buffers[inputs.size() - i - 1] = inputs[i]->buffer;
    }
    thneed->execute(input_buffers, output);
  }
}

static const int FEATURE_LEN = 512;
static const int HISTORY_BUFFER_LEN = 99;
static const int OUTPUT_SIZE = 5992;
static const int LATERAL_CONTROL_PARAMS_LEN = 2;
static const int PREV_DESIRED_CURVS_LEN = 1 * (HISTORY_BUFFER_LEN + 1);

static ThneedModel* thneed = NULL;
static jfloat* outputs = NULL;
static jint output_len = 0;
static float* zero_buf = NULL;
static float* features_buf = NULL;
static float* prev_curvs_buf = NULL;
static const int zero_len = 1024 / 4;
static const int features_len = HISTORY_BUFFER_LEN * FEATURE_LEN;

static const int kImgLen = 1572864 / 4;
static const int kDesireLen = 3200 / 4;

extern "C" {
// process goes down, so the promised fallback to ONNX would not happen.
static bool openclReady()
{
  if (opencl_library == NULL) {
    __android_log_print(ANDROID_LOG_ERROR, "thneed", "OpenCL not loaded — see dlopen warnings above");
    return false;
  }
  struct {
    const char* name;
    void* ptr;
  } required[] = {
      {"clGetPlatformIDs", (void*)p_clGetPlatformIDs},
      {"clGetDeviceIDs", (void*)p_clGetDeviceIDs},
      {"clCreateContext", (void*)p_clCreateContext},
      {"clCreateProgramWithBinary", (void*)p_clCreateProgramWithBinary},
      {"clBuildProgram", (void*)p_clBuildProgram},
      {"clCreateBuffer", (void*)p_clCreateBuffer},
  };
  bool ok = true;
  for (auto& r : required) {
    if (r.ptr == NULL) {
      __android_log_print(ANDROID_LOG_ERROR, "thneed", "dlsym(%s) = NULL", r.name);
      ok = false;
    }
  }
  if (ok) {
    __android_log_print(ANDROID_LOG_INFO, "thneed", "OpenCL resolved at %p", opencl_library);
  }
  return ok;
}

JNIEXPORT jboolean JNICALL Java_adas_app_vision_SupercomboThneedRunner_nativeInit(JNIEnv* env, jclass,
                                                                                  jbyteArray modelData, jint outLen)
{
  if (!openclReady()) {
    return JNI_FALSE;
  }
  if (thneed != NULL) {
    for (int i = 0; i < features_len; i++)
      features_buf[i] = 0;
    for (int i = 0; i < PREV_DESIRED_CURVS_LEN; i++)
      prev_curvs_buf[i] = 0;
    __android_log_print(ANDROID_LOG_INFO, "thneed", "nativeInit again — recurrent state cleared");
    return JNI_TRUE;
  }
  output_len = outLen;
  outputs = new jfloat[output_len];
  zero_buf = new float[zero_len];
  features_buf = new float[features_len];
  prev_curvs_buf = new float[PREV_DESIRED_CURVS_LEN];
  for (int i = 0; i < zero_len; i++)
    zero_buf[i] = 0;
  for (int i = 0; i < features_len; i++)
    features_buf[i] = 0;
  for (int i = 0; i < PREV_DESIRED_CURVS_LEN; i++)
    prev_curvs_buf[i] = 0;

  jbyte* bytes = env->GetByteArrayElements(modelData, 0);
  thneed = new ThneedModel((uint8_t*)bytes, outputs, output_len, 0, false, NULL);
  env->ReleaseByteArrayElements(modelData, bytes, JNI_ABORT);
  __android_log_print(ANDROID_LOG_INFO, "thneed", "loaded, output_len=%d features=%d", output_len, features_len);
  return thneed != NULL ? JNI_TRUE : JNI_FALSE;
}

namespace {

// Model frame size: 6 planes of 256x128. One place for the kernel, the buffers and the JNI length checks.
const int kWarpOutW = 256;
const int kWarpOutH = 128;

const char* kWarpSource = R"CLC(
inline float sample_bilinear(__global const uchar* px, int w, int h, float sx, float sy) {
  sx = clamp(sx, 0.0f, (float)(w - 1));
  sy = clamp(sy, 0.0f, (float)(h - 1));
  int x0 = (int)floor(sx);
  int y0 = (int)floor(sy);
  int x1 = min(x0 + 1, w - 1);
  int y1 = min(y0 + 1, h - 1);
  float fx = sx - (float)x0;
  float fy = sy - (float)y0;
  float v00 = (float)px[y0 * w + x0];
  float v10 = (float)px[y0 * w + x1];
  float v01 = (float)px[y1 * w + x0];
  float v11 = (float)px[y1 * w + x1];
  float top = v00 + (v10 - v00) * fx;
  float bot = v01 + (v11 - v01) * fx;
  return top + (bot - top) * fy;
}

inline float sample_proj(__global const uchar* px, int w, int h, __global const float* m, int x, int y) {
  float fx = (float)x;
  float fy = (float)y;
  float X = m[0] * fx + m[1] * fy + m[2];
  float Y = m[3] * fx + m[4] * fy + m[5];
  float W = m[6] * fx + m[7] * fy + m[8];
  if (fabs(W) < 1e-8f) return 0.0f;
  return sample_bilinear(px, w, h, X / W, Y / W);
}

__kernel void warp_yuv6(__global const uchar* y, __global const uchar* u, __global const uchar* v,
                        int width, int height,
                        __global const float* m, __global const float* m_uv,
                        __global float* out) {
  int i = get_global_id(0);
  int j = get_global_id(1);
  int ww = get_global_size(0);
  int hh = get_global_size(1);
  int plane = ww * hh;
  int idx = j * ww + i;
  int uv_w = width / 2;
  int uv_h = height / 2;
  int mx0 = 2 * i;
  int my0 = 2 * j;
  out[0 * plane + idx] = sample_proj(y, width, height, m, mx0,     my0);
  out[1 * plane + idx] = sample_proj(y, width, height, m, mx0,     my0 + 1);
  out[2 * plane + idx] = sample_proj(y, width, height, m, mx0 + 1, my0);
  out[3 * plane + idx] = sample_proj(y, width, height, m, mx0 + 1, my0 + 1);
  out[4 * plane + idx] = sample_proj(u, uv_w, uv_h, m_uv, i, j);
  out[5 * plane + idx] = sample_proj(v, uv_w, uv_h, m_uv, i, j);
}
)CLC";

void mul3(const float a[9], const float b[9], float r[9])
{
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      r[i * 3 + j] = a[i * 3 + 0] * b[0 * 3 + j] + a[i * 3 + 1] * b[1 * 3 + j] + a[i * 3 + 2] * b[2 * 3 + j];
}

/// The same half-pixel correction as in the CPU version: the chroma planes step twice as coarsely.
void transform_scale_buffer(const float m[9], float s, float out[9])
{
  const float inv_s = 1.0f / s;
  const float transform_out[9] = {inv_s, 0, 0.5f, 0, inv_s, 0.5f, 0, 0, 1};
  const float transform_in[9] = {s, 0, -0.5f * s, 0, s, -0.5f * s, 0, 0, 1};
  float tmp[9];
  mul3(m, transform_out, tmp);
  mul3(transform_in, tmp, out);
}

class GpuWarp {
public:
  static GpuWarp& instance()
  {
    static GpuWarp warp;
    return warp;
  }

  /// Both warps in one go. Returns false if the GPU is unavailable — the caller falls back to the CPU.
  bool run(const uint8_t* y, const uint8_t* u, const uint8_t* v, int width, int height, const float* m_narrow,
           const float* m_wide, float* out_narrow, float* out_wide)
  {
    if (!ensure(width, height))
      return false;

    const size_t y_size = (size_t)width * height;
    const size_t uv_size = (size_t)(width / 2) * (height / 2);
    if ((*p_clEnqueueWriteBuffer)(queue_, buf_y_, CL_FALSE, 0, y_size, y, 0, NULL, NULL) != CL_SUCCESS ||
        (*p_clEnqueueWriteBuffer)(queue_, buf_u_, CL_FALSE, 0, uv_size, u, 0, NULL, NULL) != CL_SUCCESS ||
        (*p_clEnqueueWriteBuffer)(queue_, buf_v_, CL_FALSE, 0, uv_size, v, 0, NULL, NULL) != CL_SUCCESS)
      return false;

    if (!enqueue(m_narrow, buf_m_a_, buf_muv_a_, buf_out_a_, width, height) ||
        !enqueue(m_wide, buf_m_b_, buf_muv_b_, buf_out_b_, width, height))
      return false;

    const size_t out_bytes = (size_t)6 * kPlane * sizeof(float);
    if ((*p_clEnqueueReadBuffer)(queue_, buf_out_a_, CL_FALSE, 0, out_bytes, out_narrow, 0, NULL, NULL) != CL_SUCCESS ||
        (*p_clEnqueueReadBuffer)(queue_, buf_out_b_, CL_TRUE, 0, out_bytes, out_wide, 0, NULL, NULL) != CL_SUCCESS)
      return false;
    return (*p_clFinish)(queue_) == CL_SUCCESS;
  }

private:
  static const int kOutW = kWarpOutW;
  static const int kOutH = kWarpOutH;
  static const int kPlane = kOutW * kOutH;

  bool enqueue(const float* m, cl_mem buf_m, cl_mem buf_muv, cl_mem buf_out, int width, int height)
  {
    float m_uv[9];
    transform_scale_buffer(m, 0.5f, m_uv);
    if ((*p_clEnqueueWriteBuffer)(queue_, buf_m, CL_FALSE, 0, 9 * sizeof(float), m, 0, NULL, NULL) != CL_SUCCESS ||
        (*p_clEnqueueWriteBuffer)(queue_, buf_muv, CL_FALSE, 0, 9 * sizeof(float), m_uv, 0, NULL, NULL) != CL_SUCCESS)
      return false;

    cl_int err = CL_SUCCESS;
    err |= (*p_clSetKernelArg)(kernel_, 0, sizeof(cl_mem), &buf_y_);
    err |= (*p_clSetKernelArg)(kernel_, 1, sizeof(cl_mem), &buf_u_);
    err |= (*p_clSetKernelArg)(kernel_, 2, sizeof(cl_mem), &buf_v_);
    err |= (*p_clSetKernelArg)(kernel_, 3, sizeof(int), &width);
    err |= (*p_clSetKernelArg)(kernel_, 4, sizeof(int), &height);
    err |= (*p_clSetKernelArg)(kernel_, 5, sizeof(cl_mem), &buf_m);
    err |= (*p_clSetKernelArg)(kernel_, 6, sizeof(cl_mem), &buf_muv);
    err |= (*p_clSetKernelArg)(kernel_, 7, sizeof(cl_mem), &buf_out);
    if (err != CL_SUCCESS)
      return false;

    const size_t global[2] = {(size_t)kOutW, (size_t)kOutH};
    const size_t local[2] = {16, 4};
    return (*p_clEnqueueNDRangeKernel)(queue_, kernel_, 2, NULL, global, local, 0, NULL, NULL) == CL_SUCCESS;
  }

  bool ensure(int width, int height)
  {
    if (opencl_library == NULL)
      return false;
    if (ready_ && width == width_ && height == height_)
      return true;
    if (failed_)
      return false;

    if (context_ == NULL) {
      cl_int err = CL_SUCCESS;
      device_ = cl_get_device_id(CL_DEVICE_TYPE_DEFAULT);
      context_ = (*p_clCreateContext)(NULL, 1, &device_, NULL, NULL, &err);
      if (context_ == NULL) {
        failed_ = true;
        return false;
      }
      cl_command_queue_properties props[3] = {CL_QUEUE_PROPERTIES, 0, 0};
      queue_ = (*p_clCreateCommandQueueWithProperties)(context_, device_, props, &err);
      if (queue_ == NULL) {
        failed_ = true;
        return false;
      }
      cl_program program = cl_program_from_source(context_, device_, kWarpSource);
      kernel_ = (*p_clCreateKernel)(program, "warp_yuv6", &err);
      if (kernel_ == NULL) {
        __android_log_print(ANDROID_LOG_ERROR, "GpuWarp", "kernel did not build, staying on the CPU");
        failed_ = true;
        return false;
      }
    }

    release_frame_buffers();
    const size_t y_size = (size_t)width * height;
    const size_t uv_size = (size_t)(width / 2) * (height / 2);
    const size_t out_bytes = (size_t)6 * kPlane * sizeof(float);
    cl_int err = CL_SUCCESS;
    buf_y_ = (*p_clCreateBuffer)(context_, CL_MEM_READ_ONLY, y_size, NULL, &err);
    buf_u_ = (*p_clCreateBuffer)(context_, CL_MEM_READ_ONLY, uv_size, NULL, &err);
    buf_v_ = (*p_clCreateBuffer)(context_, CL_MEM_READ_ONLY, uv_size, NULL, &err);
    buf_m_a_ = (*p_clCreateBuffer)(context_, CL_MEM_READ_ONLY, 9 * sizeof(float), NULL, &err);
    buf_muv_a_ = (*p_clCreateBuffer)(context_, CL_MEM_READ_ONLY, 9 * sizeof(float), NULL, &err);
    buf_m_b_ = (*p_clCreateBuffer)(context_, CL_MEM_READ_ONLY, 9 * sizeof(float), NULL, &err);
    buf_muv_b_ = (*p_clCreateBuffer)(context_, CL_MEM_READ_ONLY, 9 * sizeof(float), NULL, &err);
    buf_out_a_ = (*p_clCreateBuffer)(context_, CL_MEM_WRITE_ONLY, out_bytes, NULL, &err);
    buf_out_b_ = (*p_clCreateBuffer)(context_, CL_MEM_WRITE_ONLY, out_bytes, NULL, &err);
    if (buf_y_ == NULL || buf_u_ == NULL || buf_v_ == NULL || buf_out_a_ == NULL || buf_out_b_ == NULL) {
      failed_ = true;
      return false;
    }
    width_ = width;
    height_ = height;
    ready_ = true;
    __android_log_print(ANDROID_LOG_INFO, "GpuWarp", "GPU warp ready, frame %dx%d", width, height);
    return true;
  }

  void release_frame_buffers()
  {
    cl_mem all[] = {buf_y_, buf_u_, buf_v_, buf_m_a_, buf_muv_a_, buf_m_b_, buf_muv_b_, buf_out_a_, buf_out_b_};
    for (cl_mem m : all)
      if (m != NULL)
        (*p_clReleaseMemObject)(m);
    buf_y_ = buf_u_ = buf_v_ = NULL;
    buf_m_a_ = buf_muv_a_ = buf_m_b_ = buf_muv_b_ = NULL;
    buf_out_a_ = buf_out_b_ = NULL;
  }

  cl_context context_ = NULL;
  cl_command_queue queue_ = NULL;
  cl_device_id device_ = NULL;
  cl_kernel kernel_ = NULL;
  cl_mem buf_y_ = NULL, buf_u_ = NULL, buf_v_ = NULL;
  cl_mem buf_m_a_ = NULL, buf_muv_a_ = NULL, buf_m_b_ = NULL, buf_muv_b_ = NULL;
  cl_mem buf_out_a_ = NULL, buf_out_b_ = NULL;
  int width_ = 0, height_ = 0;
  bool ready_ = false;
  bool failed_ = false;
};

}  // namespace

JNIEXPORT jstring JNICALL Java_adas_app_vision_SupercomboThneedRunner_nativeClInfo(JNIEnv* env, jclass)
{
  char out[2048];
  if (opencl_library == NULL) {
    snprintf(out, sizeof(out), "{\"opencl\": false, \"reason\": \"libOpenCL did not load\"}");
    return env->NewStringUTF(out);
  }

  if (p_clGetDeviceInfo == NULL) {
    snprintf(out, sizeof(out), "{\"opencl\": false, \"reason\": \"clGetDeviceInfo not found\"}");
    return env->NewStringUTF(out);
  }

  cl_device_id device = cl_get_device_id(CL_DEVICE_TYPE_DEFAULT);
  if (device == NULL) {
    snprintf(out, sizeof(out), "{\"opencl\": false, \"reason\": \"no OpenCL device found\"}");
    return env->NewStringUTF(out);
  }

  char name[256] = {0};
  char version[256] = {0};
  // The extension string on Adreno is about two kilobytes, but running out of buffer here is
  // especially nasty: clGetDeviceInfo returns an error, leaves the buffer zeroed, and cl_khr_fp16
  // ends up "absent" on a device that has it. So we ask for the size and check the result.
  std::string extensions;
  size_t max_group = 0;
  cl_uint pitch_align = 0;
  cl_ulong local_mem = 0;
  (*p_clGetDeviceInfo)(device, CL_DEVICE_NAME, sizeof(name), name, NULL);
  (*p_clGetDeviceInfo)(device, CL_DEVICE_VERSION, sizeof(version), version, NULL);
  size_t ext_size = 0;
  if ((*p_clGetDeviceInfo)(device, CL_DEVICE_EXTENSIONS, 0, NULL, &ext_size) == CL_SUCCESS && ext_size > 0) {
    extensions.resize(ext_size);
    if ((*p_clGetDeviceInfo)(device, CL_DEVICE_EXTENSIONS, ext_size, &extensions[0], NULL) != CL_SUCCESS)
      extensions.clear();
  }
  (*p_clGetDeviceInfo)(device, CL_DEVICE_MAX_WORK_GROUP_SIZE, sizeof(max_group), &max_group, NULL);
  (*p_clGetDeviceInfo)(device, CL_DEVICE_IMAGE_PITCH_ALIGNMENT, sizeof(pitch_align), &pitch_align, NULL);
  (*p_clGetDeviceInfo)(device, CL_DEVICE_LOCAL_MEM_SIZE, sizeof(local_mem), &local_mem, NULL);

  const bool fp16 = extensions.find("cl_khr_fp16") != std::string::npos;
  const bool img_from_buf = extensions.find("cl_khr_image2d_from_buffer") != std::string::npos;
  // The alignment is declared in pixels, and a pixel here is RGBA half — eight bytes.
  const int pitch_bytes = (int)pitch_align * 8;

  snprintf(out, sizeof(out),
           "{\"opencl\": true, \"device\": \"%s\", \"version\": \"%s\", \"fp16\": %s, "
           "\"image2d_from_buffer\": %s, \"pitch_align_px\": %u, \"pitch_align_bytes\": %d, "
           "\"max_work_group\": %zu, \"local_mem\": %llu}",
           name, version, fp16 ? "true" : "false", img_from_buf ? "true" : "false", pitch_align, pitch_bytes, max_group,
           (unsigned long long)local_mem);
  return env->NewStringUTF(out);
}

JNIEXPORT jboolean JNICALL Java_adas_app_vision_ModelCalibWarp_nativeWarpPairGpu(
    JNIEnv* env, jclass, jbyteArray yArr, jbyteArray uArr, jbyteArray vArr, jint width, jint height,
    jfloatArray mNarrow, jfloatArray mWide, jfloatArray outNarrow, jfloatArray outWide)
{
  if (!yArr || !uArr || !vArr || !mNarrow || !mWide || !outNarrow || !outWide || width <= 0 || height <= 0)
    return JNI_FALSE;

  const jsize uv_len = (width / 2) * (height / 2);
  const jsize out_len = 6 * kWarpOutW * kWarpOutH;
  if (env->GetArrayLength(yArr) < (jsize)width * height || env->GetArrayLength(uArr) < uv_len ||
      env->GetArrayLength(vArr) < uv_len || env->GetArrayLength(mNarrow) < 9 || env->GetArrayLength(mWide) < 9 ||
      env->GetArrayLength(outNarrow) < out_len || env->GetArrayLength(outWide) < out_len) {
    __android_log_print(ANDROID_LOG_ERROR, "GpuWarp", "arrays are the wrong size — warp skipped");
    return JNI_FALSE;
  }

  jbyte* y = env->GetByteArrayElements(yArr, NULL);
  jbyte* u = env->GetByteArrayElements(uArr, NULL);
  jbyte* v = env->GetByteArrayElements(vArr, NULL);
  jfloat* mn = env->GetFloatArrayElements(mNarrow, NULL);
  jfloat* mw = env->GetFloatArrayElements(mWide, NULL);
  jfloat* on = env->GetFloatArrayElements(outNarrow, NULL);
  jfloat* ow = env->GetFloatArrayElements(outWide, NULL);

  bool ok =
      y && u && v && mn && mw && on && ow &&
      GpuWarp::instance().run((const uint8_t*)y, (const uint8_t*)u, (const uint8_t*)v, width, height, mn, mw, on, ow);

  if (y)
    env->ReleaseByteArrayElements(yArr, y, JNI_ABORT);
  if (u)
    env->ReleaseByteArrayElements(uArr, u, JNI_ABORT);
  if (v)
    env->ReleaseByteArrayElements(vArr, v, JNI_ABORT);
  if (mn)
    env->ReleaseFloatArrayElements(mNarrow, mn, JNI_ABORT);
  if (mw)
    env->ReleaseFloatArrayElements(mWide, mw, JNI_ABORT);
  if (on)
    env->ReleaseFloatArrayElements(outNarrow, on, ok ? 0 : JNI_ABORT);
  if (ow)
    env->ReleaseFloatArrayElements(outWide, ow, ok ? 0 : JNI_ABORT);
  return ok ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT jboolean JNICALL Java_adas_app_vision_SupercomboThneedRunner_nativeSave(JNIEnv* env, jclass, jstring jpath,
                                                                                  jboolean binaries)
{
  if (thneed == NULL)
    return JNI_FALSE;
  const char* path = env->GetStringUTFChars(jpath, 0);
  const bool ok = thneed->saveTo(path, binaries == JNI_TRUE);
  env->ReleaseStringUTFChars(jpath, path);
  return ok ? JNI_TRUE : JNI_FALSE;
}

JNIEXPORT jfloat JNICALL Java_adas_app_vision_SupercomboThneedRunner_nativeExecute(JNIEnv* env, jclass,
                                                                                   jfloatArray input,
                                                                                   jfloatArray output)
{
  if (thneed == NULL)
    return -1.0f;
  jfloat* in = env->GetFloatArrayElements(input, 0);

  float* imgs = &in[0];
  float* big_imgs = &in[kImgLen];
  float* desire = &in[kImgLen * 2];
  float* lat_params = &in[kImgLen * 2 + kDesireLen];

  thneed->setInputBuffer("input_imgs", imgs, kImgLen);
  thneed->setInputBuffer("big_input_imgs", big_imgs, kImgLen);
  thneed->setInputBuffer("desire", desire, kDesireLen);
  thneed->setInputBuffer("traffic_convention", zero_buf, 8 / 4);
  thneed->setInputBuffer("lateral_control_params", lat_params, LATERAL_CONTROL_PARAMS_LEN);
  thneed->setInputBuffer("prev_desired_curv", prev_curvs_buf, PREV_DESIRED_CURVS_LEN);
  thneed->setInputBuffer("nav_features", zero_buf, 1024 / 4);
  thneed->setInputBuffer("nav_instructions", zero_buf, 600 / 4);
  thneed->setInputBuffer("features_buffer", features_buf, features_len);

  double t0 = millis_since_boot();
  thneed->execute();
  float ms = (float)(millis_since_boot() - t0);

  env->ReleaseFloatArrayElements(input, in, JNI_ABORT);

  // Recurrence: shift the history by one frame and append fresh features from the output tail.
  std::memmove(&features_buf[0], &features_buf[FEATURE_LEN], sizeof(float) * FEATURE_LEN * (HISTORY_BUFFER_LEN - 1));
  std::memcpy(&features_buf[FEATURE_LEN * (HISTORY_BUFFER_LEN - 1)], &outputs[OUTPUT_SIZE],
              sizeof(float) * FEATURE_LEN);

  // and leaving a half-written float at the boundary. Shift by elements.
  std::memmove(&prev_curvs_buf[0], &prev_curvs_buf[1], sizeof(float) * (PREV_DESIRED_CURVS_LEN - 1));
  prev_curvs_buf[PREV_DESIRED_CURVS_LEN - 1] = outputs[5990];

  env->SetFloatArrayRegion(output, 0, output_len, outputs);
  return ms;
}
}
