/*

    Copyright Eli Dupree and Isaac Dupree, 2013

    This file is part of Lasercake.

    Lasercake is free software: you can redistribute it and/or modify
    it under the terms of the GNU Affero General Public License as
    published by the Free Software Foundation, either version 3 of the
    License, or (at your option) any later version.

    Lasercake is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Affero General Public License for more details.

    You should have received a copy of the GNU Affero General Public License
    along with Lasercake.  If not, see <http://www.gnu.org/licenses/>.

*/

#include <streambuf>
#include <string.h>
#include <unistd.h>
#include <errno.h>

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

  log::log() : os_(&streambuf_) { os_.imbue(std::locale::classic()); os_ << std::boolalpha; }
  log::~log() {}
}
