
#include <streambuf>
#include <string.h>
#include <unistd.h>

#include <QtCore/QThread>

#include "config.hpp"

// Use POSIX write() rather than C stdio to make sure
// there aren't any locks/mutexes/global state hiding out anywhere
// that thread cancellation could mess up.

static const int log_output_fd = 2 /*stderr, STDERR_FILENO*/;

static void do_write(int fd, const char* ptr, size_t n) {
  int patience = 10;
  while(n > 0 && patience > 0) {
    const ssize_t ret = write(fd, ptr, n);
    if(ret == -1) {
      if(!(errno == EINTR || errno == EAGAIN)) {
        break; //give up
      }
    }
    else {
      ptr += ret;
      n -= ret;
    }
    QThread::yieldCurrentThread();
    --patience;
  }
}

namespace logger_impl {
  void log_buf::write_buf_() {
    const std::streamsize bufn = pptr() - pbase();
    do_write(log_output_fd, pbase(), bufn);
    pbump(-bufn);
  }
  std::streamsize log_buf::xsputn(const char* s, std::streamsize n) {
    if(pptr() + n > epptr()) {
      write_buf_();
      if(n >= bufsize_) {
        do_write(log_output_fd, s, n);
        return n;
      }
    }
    memcpy(pptr(), s, n);
    pbump(n);
    return n;
  }
  log_buf::int_type log_buf::overflow(int_type ch) {
    if(pptr() >= epptr()) {
      write_buf_();
    }
    if(ch != traits_type::eof()) {
      *pptr() = ch;
      pbump(1);
    }
    return traits_type::not_eof(ch); // which means "success"
  }
}
