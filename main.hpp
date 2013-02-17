/*

    Copyright Eli Dupree and Isaac Dupree, 2011, 2012

    This file is part of Lasercake.

    Lasercake is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Lasercake is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Lasercake.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef LASERCAKE_MAIN_HPP__
#define LASERCAKE_MAIN_HPP__

#include <QtOpenGL/QGLWidget>
#include <QtCore/QThread>
#include <QtCore/QMutex>
#include <QtCore/QWaitCondition>
#include <QtGui/QKeyEvent>

#include <set>

#include "gl_data_preparation.hpp"
#include "input_representation.hpp"
#include "world.hpp"
#include "gl_rendering.hpp"

typedef abstract_gl_data gl_data_t;

//is a pointer to avoid copying around all that data
typedef shared_ptr<gl_data_t> gl_data_ptr_t;

// TODO UNITS use units for this:
typedef int64_t microseconds_t;

struct frame_output_t {
  gl_data_ptr_t gl_data_ptr;
  microseconds_t microseconds_for_drawing;
  microseconds_t microseconds_for_simulation;
  std::string extra_debug_info;
};

using input_representation::input_news_t;

struct config_struct {
  std::string scenario;
  bool crazy_lasers;
  distance view_radius;
  bool have_gui;
  bool run_drawing_code;
  bool initially_drawing_debug_stuff;
  int64_t exit_after_frames;
  bool use_opengl_thread;
  bool use_simulation_thread;
};
inline std::ostream& operator<<(std::ostream& os, config_struct const& config) {
  return os << '{'
    << "scenario=" << config.scenario
    << "; crazy_lasers=" << config.crazy_lasers
    << "; view_radius=" << config.view_radius
    << "; have_gui=" << config.have_gui
    << "; run_drawing_code=" << config.run_drawing_code
    << "; initially_drawing_debug_stuff=" << config.initially_drawing_debug_stuff
    << "; exit_after_frames=" << config.exit_after_frames
    << "; use_opengl_thread=" << config.use_opengl_thread
    << "; use_simulation_thread=" << config.use_simulation_thread
    << '}';
}

// for use in its own QThread
class LasercakeSimulator : public QObject {
  Q_OBJECT

public:
  explicit LasercakeSimulator(QObject* parent = 0);

  Q_INVOKABLE void init(worldgen_function_t worldgen, config_struct config);
  Q_INVOKABLE void new_input_as_of(time_unit moment, input_news_t new_input);
  //actually_prepare_graphics=false still sends frame_output_ready with useful debug info.
  Q_INVOKABLE void prepare_graphics(input_news_t input_since_last_prepare, distance view_radius, bool actually_prepare_graphics);

Q_SIGNALS:
  void sim_frame_done(time_unit moment);
  void frame_output_ready(time_unit moment, frame_output_t output);

private:
  shared_ptr<world> world_ptr_;
  microseconds_t microseconds_last_sim_frame_took_; //hack
  shared_ptr<view_on_the_world> view_ptr_;
  object_identifier currently_focused_object_;
};

struct gl_thread_data_t {
  QMutex gl_data_lock;
  QWaitCondition wait_for_instruction;

  // access protected by the mutex:
  bool quit_now;
  uint64_t revision;
  gl_data_ptr_t current_data;
  microseconds_t microseconds_last_gl_render_took;
  QSize viewport_size;
};

class LasercakeGLWidget;
// A Qt-signals-based event loop couldn't do quite what I wanted here
// (insufficient info/manipulation of the sequence of queued signals),
// so use threading abstractions + no event loop.
class LasercakeGLThread : public QThread {
  Q_OBJECT

public:
  shared_ptr<gl_thread_data_t> gl_thread_data_;
  // after thread starts, only thread accesses these:
  LasercakeGLWidget* gl_widget_;
  uint64_t last_revision_;
  microseconds_t microseconds_this_gl_render_took_;
  microseconds_t gl_render(gl_data_ptr_t& gl_data_ptr, LasercakeGLWidget& gl_widget, QSize viewport_size);
  gl_renderer gl_renderer_;
protected:
  void run() override;
};


class LasercakeGLWidget : public QGLWidget {
  Q_OBJECT

public:
  explicit LasercakeGLWidget(bool use_separate_gl_thread, QWidget* parent = 0);
  void update_gl_data(gl_data_ptr_t data);
  microseconds_t get_last_gl_render_microseconds();
  input_representation::input_news_t get_input_news()const;
  // doesn't clear the keys_currently_pressed, only the history:
  void clear_input_news();
  void toggle_fullscreen();
  void toggle_fullscreen(bool fullscreen);

Q_SIGNALS:
  void key_changed(input_representation::key_change_t);

protected:
  bool event(QEvent*) override;
  //void keyPressEvent(QKeyEvent*) override;
  //void keyReleaseEvent(QKeyEvent*) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void mousePressEvent(QMouseEvent* event) override;
  void focusOutEvent(QFocusEvent*) override;
  void resizeEvent(QResizeEvent*) override;
  void paintEvent(QPaintEvent*) override;
  void closeEvent(QCloseEvent*) override;

private Q_SLOTS:
  void prepare_to_cleanly_close_();

private:
  void key_change_(QKeyEvent* event, bool pressed);
  void invoke_render_(); //precondition: you incremented gl_thread_data_->revision
  void grab_input_();
  void ungrab_input_();

  typedef int qt_key_type_;
  // e.g. in case there can be multiple shift keys pressed at once
  // (two of them on a regular keyboard... or two USB keyboards plugged in,
  //  for that matter!)
  std::multiset<qt_key_type_> keys_currently_pressed_;
  input_representation::key_activity_t input_rep_key_activity_;
  input_representation::keys_currently_pressed_t input_rep_keys_currently_pressed_;
  input_representation::mouse_displacement_t input_rep_mouse_displacement_;
  QPoint global_cursor_pos_;
  bool input_is_grabbed_;
  bool use_separate_gl_thread_;
  LasercakeGLThread thread_;
  shared_ptr<gl_thread_data_t> gl_thread_data_;
  bool has_quit_;
};


class LasercakeController : public QObject {
  Q_OBJECT

public:
  explicit LasercakeController(config_struct config, QObject* parent = 0);

public Q_SLOTS:
  void output_new_frame(time_unit moment, frame_output_t output);
  void key_changed(input_representation::key_change_t);

private:
  bool invoke_simulation_step_();

  config_struct config_;
  boost::scoped_ptr<LasercakeGLWidget> gl_widget_;
  boost::scoped_ptr<QThread> simulator_thread_;
  boost::scoped_ptr<LasercakeSimulator> simulator_;
  microseconds_t monotonic_microseconds_at_beginning_of_frame_;
  microseconds_t monotonic_microseconds_at_beginning_of_ten_frame_block_;
  microseconds_t monotonic_microseconds_at_beginning_of_hundred_frame_block_;
  int64_t frame_;
  time_unit game_time_;
  bool paused_;
  int64_t steps_queued_to_do_while_paused_;
};

#endif
