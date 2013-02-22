/*

    Copyright Eli Dupree and Isaac Dupree, 2011, 2012, 2013

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

// BOOST_SYSTEM_NO_DEPRECATED prevents some deprecated members with global constructors.
#ifndef BOOST_SYSTEM_NO_DEPRECATED
#define BOOST_SYSTEM_NO_DEPRECATED
#endif

#include "config.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <QtGui/QApplication>
#include <QtGui/QDesktopWidget>
#include <QtCore/QMetaEnum>
#include <QtCore/QLocale>
#include <QtGui/QFontDatabase>

#include <iomanip>
#include <sstream>
#include <locale>

#ifdef LASERCAKE_HAVE_SYS_RESOURCE_H
#include <sys/resource.h>
#endif
#ifndef BOOST_CHRONO_HEADER_ONLY
#define BOOST_CHRONO_HEADER_ONLY
#endif
#include <boost/chrono.hpp>
#include <boost/chrono/process_cpu_clocks.hpp>
#include <boost/chrono/thread_clock.hpp>

#include <boost/scope_exit.hpp>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <boost/program_options.hpp>
#pragma GCC diagnostic pop

#include "main.hpp"

#include "world.hpp"
#include "specific_worlds.hpp"
#include "specific_object_types.hpp"

#include "tests/test_main.hpp"

namespace /* anonymous */ {

namespace chrono = boost::chrono;

int64_t get_this_process_mem_usage_megabytes() {
#if defined(LASERCAKE_HAVE_SYS_RESOURCE_H)
  struct rusage ru;
  getrusage(RUSAGE_SELF, &ru);
  #if defined(__APPLE__) || defined(__MACOSX__)
    return ru.ru_maxrss / (1024*1024);
  #else
    return ru.ru_maxrss / 1024;
  #endif
#else
  return 0;
#endif
}
microseconds_t get_this_thread_microseconds() {
#if defined(BOOST_CHRONO_HAS_THREAD_CLOCK)
  return chrono::duration_cast<chrono::microseconds>(chrono::thread_clock::now().time_since_epoch()).count();
#else
  return 0;
#endif
}
microseconds_t get_this_process_microseconds() {
#if defined(BOOST_CHRONO_HAS_PROCESS_CLOCKS)
  return chrono::duration_cast<chrono::microseconds>(chrono::process_real_cpu_clock::now().time_since_epoch()).count();
#else
  return 0;
#endif
}
microseconds_t get_monotonic_microseconds() {
  return chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now().time_since_epoch()).count();
}

// Usage example:
// LOG << std::setw(6) << (ostream_bundle() << "foo" << 564) << std::endl;
struct ostream_bundle : std::ostream {
  template<typename T> ostream_bundle& operator<<(T const& t) { ss_ << t; return *this; }
  std::string str() { return ss_.str(); }
private:
  std::stringstream ss_;
};
std::ostream& operator<<(ostream_bundle& os, ostream_bundle& b) { return os << b.str(); }
std::ostream& operator<<(std::ostream& os, ostream_bundle& b) { return os << b.str(); }

// show_decimal(1234567, 1000, 10) --> "1234.5"
template<typename Integral, typename Integral2>
std::string show_decimal(Integral us, Integral2 divisor, int places, std::locale const& locale = std::locale()) {
  Integral divisordivisor = 1;
  for(int i = 0; i < places; ++i) { divisordivisor *= 10; }
  
  return (ostream_bundle()
    << (us / divisor)
    << std::use_facet< std::numpunct<char> >(locale).decimal_point()
    << std::setfill('0') << std::setw(places) << std::abs(us / (divisor / divisordivisor) % divisordivisor)
  ).str();
}

std::string show_microseconds(microseconds_t us) {
  return show_decimal(us, 1000, 1);
}
std::string show_microseconds_per_frame_as_fps(microseconds_t monotonic_microseconds_for_frame) {
  std::string frames_per_second_str = " inf ";
  if(monotonic_microseconds_for_frame > 0) {
    const int64_t frames_per_kilosecond = 1000000000 / monotonic_microseconds_for_frame;
    frames_per_second_str = show_decimal(frames_per_kilosecond, 1000, 2);
  }
  return frames_per_second_str;
}




object_identifier init_test_world_and_return_our_robot(
      world& w, bool crazy_lasers, bool log_microseconds = true) {
  const vector3<distance> laser_loc = world_center_fine_coords
    + vector3<distance>(10LL*tile_width + 2*fine_distance_units,
                           10LL*tile_width + 2*fine_distance_units,
                           10LL*tile_width + 2*fine_distance_units);
  const shared_ptr<robot> baz (new robot(
    laser_loc - vector3<distance>(0, 0, tile_width*2),
    vector3<distance>((5<<9)*fine_distance_units, (3<<9)*fine_distance_units, 0 /*TODO UNITS make class vector3_direction*/)));
  const object_identifier robot_id = w.try_create_object(baz); // we just assume that this works
  const shared_ptr<autorobot> aur (new autorobot(
    get_min_containing_tile_coordinates(laser_loc - vector3<distance>(tile_width*4,tile_width*4,tile_width*2)),
    vector3<distance>((5<<9)*fine_distance_units, (3<<9)*fine_distance_units, 0 /*TODO UNITS make class vector3_direction*/)));
  w.try_create_object(aur); // we just assume that this works

  if(crazy_lasers) {
    const shared_ptr<laser_emitter> foo (new laser_emitter(
      laser_loc,
      vector3<distance>(5*fine_distance_units, 3*fine_distance_units, 1*fine_distance_units /*TODO UNITS make class vector3_direction*/)));
    const shared_ptr<laser_emitter> bar (new laser_emitter(
      laser_loc + vector3<distance>(0,0,tile_width*2),
      vector3<distance>(5*fine_distance_units, 4*fine_distance_units, -1*fine_distance_units /*TODO UNITS make class vector3_direction*/)));
    w.try_create_object(foo);
    w.try_create_object(bar);
  }

  // HACK - TODO remove at least the second effect
  // This hack has two effects:
  // Generating all the worldblocks near the start right away
  // and
  // (BIG HACK)
  // making water (that's near enough to the start) start out ungroupable
  // which helps us e.g. start with a pillar of water in the air that's about to fall,
  // so we can test how that physics works.
  {
    if(log_microseconds) {
      LOG << "\nInit: ";
    }
    const microseconds_t microseconds_before_init = get_this_process_microseconds();

    vector<tile_location> tiles_near_start;
    // I choose these distances big enough that, as of the time of writing this comment,
    // the GLOBAL view won't have to realize any new tiles in order to make a complete cycle
    // around world-center.  This is an interim way to get rid of that annoying lag, at the cost
    // of a bit more of annoying startup time.
    const bounding_box initial_area = bounding_box::min_and_max(
      world_center_fine_coords - vector3<distance>(tile_width*80,tile_width*80,tile_width*80),
      world_center_fine_coords + vector3<distance>(tile_width*80,tile_width*80,tile_width*80)
    );
    w.ensure_realization_of_space(initial_area, FULL_REALIZATION);
    w.get_tiles_exposed_to_collision_within(tiles_near_start,
        get_tile_bbox_containing_all_tiles_intersecting_fine_bbox(initial_area));
    for (tile_location loc : tiles_near_start) {
      if (loc.stuff_at().contents() == GROUPABLE_WATER) {
        w.replace_substance(loc, GROUPABLE_WATER, UNGROUPABLE_WATER);
      }
    }

    const microseconds_t microseconds_after_init = get_this_process_microseconds();
    const microseconds_t microseconds_for_init = microseconds_after_init - microseconds_before_init;
    if(log_microseconds) {
      LOG << show_microseconds(microseconds_for_init) << " ms\n";
    }
  }

  return robot_id;
}

std::string get_world_ztree_debug_info(world const& w) {
  std::stringstream world_ztree_debug_info;
  // hack to print this debug info occasionally
  if(w.game_time_elapsed() % (5*seconds * identity(time_units / seconds))
          < (1*seconds*identity(time_units / seconds) / 15)) {
    //world_ztree_debug_info << "tiles:";
    //w.tiles_exposed_to_collision().print_debug_summary_information(world_ztree_debug_info);
    world_ztree_debug_info << "objects:";
    w.objects_exposed_to_collision().print_debug_summary_information(world_ztree_debug_info);
  }
  else {
    // zobj = ztree objects
    world_ztree_debug_info //<< "t:" << w.tiles_exposed_to_collision().size()
      << "o:" << w.objects_exposed_to_collision().size() << " zobj; ";
  }
  return world_ztree_debug_info.str();
}
} /* end anonymous namespace */


// LasercakeSimulator:
//
// When exiting the program, we implicitly roughly terminate the simulation
// thread by calling C's exit() without stopping or waiting for that thread.
//
// The thread has no side effects visible outside the
// program, so this doesn't lead to visible race conditions.
//
// (
//   This might even be safe if we were not exiting the program
//   (via e.g. simulator_thread_->terminate()):
//   The simulation thread refers to no global data
//      (except for implementations of e.g. borrowed_bitset, hmm)
//   and has no side effects
//      (except for Qt inter-thread communication; what if terminating
//       it leaves a Qt-implementation mutex in a wrong state? Hmm.
//       This could be impossiblified by using lock-free communication
//       between ours and the sim thread).
// )
LasercakeSimulator::LasercakeSimulator(QObject* parent) : QObject(parent) {}

void LasercakeSimulator::init(worldgen_function_t worldgen, config_struct config) {
  const microseconds_t microseconds_before_initing = get_this_thread_microseconds();
  world_ptr_.reset(new world(worldgen));
  const object_identifier robot_id = init_test_world_and_return_our_robot(*world_ptr_, config.crazy_lasers, config.show_frame_timing);
  currently_focused_object_ = robot_id;
  view_ptr_.reset(new view_on_the_world(world_center_fine_coords));
  view_ptr_->drawing_debug_stuff = config.initially_drawing_debug_stuff;
  const microseconds_t microseconds_after_initing = get_this_thread_microseconds();
  microseconds_last_sim_frame_took_ = microseconds_after_initing - microseconds_before_initing; //hack?
}
//TODO have each keypress have a time_unit as well.
//It implicitly computes up to the latest frame for which it
//has input, currently.
//Possibly-a-bad-idea: we could compute ahead assuming that
//the same set of keys are going to stay pressed, and backtrack
//and re-simulate if that doesn't turn out to be the case.

//for now, just assume updates are in the usual time-steps (bogus)
void LasercakeSimulator::new_input_as_of(time_unit /*moment*/, input_news_t input_news) {
  //(TODO?) we don't currently respect the given moment to simulate up to
  const microseconds_t microseconds_before_simulating = get_this_thread_microseconds();
  unordered_map<object_identifier, input_news_t> targeted_input;
  targeted_input[currently_focused_object_] = input_news;
  world_ptr_->update(targeted_input);
  const microseconds_t microseconds_after_simulating = get_this_thread_microseconds();
  microseconds_last_sim_frame_took_ = microseconds_after_simulating - microseconds_before_simulating;

  Q_EMIT sim_frame_done(world_ptr_->game_time_elapsed());
}

// TODO think about that input_news argument and the timing of when it's read etc.
void LasercakeSimulator::prepare_graphics(input_news_t input_since_last_prepare, distance view_radius, bool actually_prepare_graphics) {
  const microseconds_t microseconds_before_drawing = get_this_thread_microseconds();
  gl_data_ptr_t gl_data_ptr(new gl_data_t());
  if(actually_prepare_graphics) {
    const int32_t num_tab_presses = input_since_last_prepare.num_times_pressed("tab");
    for(int32_t i = 0; i != num_tab_presses; ++i) {
      //TODO better than O(N) (or worse, O(N * num tab presses!)
      objects_map<object>::type const& objects = world_ptr_->get_objects();
      //restrict to autonomous objs perhaps?
      object_identifier best = currently_focused_object_; //the worst best
      for(auto const& p : objects) {
        if(boost::dynamic_pointer_cast<object_with_eye_direction>(p.second)) {
          if(p.first > currently_focused_object_) {
            if(best <= currently_focused_object_ || p.first < best) {
              best = p.first;
            }
          }
          else if(best <= currently_focused_object_ && p.first < best) {
            best = p.first;
          }
        }
      }
      currently_focused_object_ = best;
    }
    //hmm
    view_ptr_->input(input_since_last_prepare);
    view_ptr_->prepare_gl_data(*world_ptr_, gl_data_preparation_config(view_radius, currently_focused_object_), *gl_data_ptr);
  }
  const microseconds_t microseconds_after_drawing = get_this_thread_microseconds();
  const microseconds_t microseconds_for_drawing = microseconds_after_drawing - microseconds_before_drawing;

  const frame_output_t output = {
    gl_data_ptr,
    microseconds_for_drawing,
    microseconds_last_sim_frame_took_,
    get_world_ztree_debug_info(*world_ptr_)
  };

  Q_EMIT frame_output_ready(world_ptr_->game_time_elapsed(), output);
}



namespace /*anonymous*/ {

// Boost doesn't appear to provide a reverse-sense bool switch, so:
boost::program_options::typed_value<bool>* bool_switch_off(bool* v = nullptr) {
  using boost::program_options::typed_value;
  typed_value<bool>* result = new typed_value<bool>(v);
  result->default_value(true);
  result->implicit_value(false);
  result->zero_tokens();
  return result;
}

} /* end anonymous namespace */

int debug_test_sim_avoiding_qt_and_trying_to_be_very_deterministic(config_struct config) {
  LOG << "Constructing worldgen\n";
  const worldgen_function_t worldgen = make_world_building_func(config.scenario);
  assert(worldgen);
  LOG << "Constructing world\n";
  world w(worldgen);
  LOG << "Initing world\n";
  const object_identifier robot_id = init_test_world_and_return_our_robot(w, config.crazy_lasers, false);
  shared_ptr<view_on_the_world> view_ptr;
  if(config.run_drawing_code) {
    LOG << "Initing graphics-prep code\n";
    view_ptr.reset(new view_on_the_world(world_center_fine_coords));
    view_ptr->drawing_debug_stuff = config.initially_drawing_debug_stuff;
  }
  LOG << "Beginning simulating\n";
  int64_t frame = 0;
  const unordered_map<object_identifier, input_news_t> there_aint_any_input;
  while(config.exit_after_frames != frame) {
    ++frame;
    w.update(there_aint_any_input);
    if(config.run_drawing_code) {
      LOG << "end sim; " << std::flush;
      {
        gl_data_ptr_t gl_data_ptr(new gl_data_t());
        view_ptr->prepare_gl_data(w, gl_data_preparation_config(config.view_radius, robot_id), *gl_data_ptr);
      }
    }
    LOG << "end frame " << frame << std::endl;
  }
  LOG << "Ending simulating\n";
  return 0;
}

int main(int argc, char *argv[])
{
  try {
    std::locale::global(std::locale(""));
  }
  catch(std::runtime_error&) {
    LOG << "Can't find your default locale; not setting locale" << std::endl;
  }
  LOG << std::boolalpha;

  config_struct config;

  {
    namespace po = boost::program_options;

    po::positional_options_description p;
    p.add("scenario", 1);

    po::options_description desc("Allowed options");
    desc.add_options()
      ("help,h", "produce help message")
      ("view-radius,v", po::value<uint32_t>()->default_value(50), "view radius, in tile_widths") //TODO - in meters?
      ("crazy-lasers,l", po::bool_switch(&config.crazy_lasers), "start with some lasers firing in lots of random directions")
      ("initially-drawing-debug-stuff,d", po::bool_switch(&config.initially_drawing_debug_stuff), "initially drawing debug stuff")
      ("scenario", po::value<std::string>(&config.scenario), "which scenario to run (also accepted with --scenario omitted)")
      ("exit-after-frames,e", po::value<int64_t>(&config.exit_after_frames)->default_value(-1), "debug: exit after n frames (negative: never)")
      ("no-gui,n", bool_switch_off(&config.have_gui), "debug: don't run the GUI")
      ("sim-only,s", bool_switch_off(&config.run_drawing_code), "debug: don't draw/render at all")
      ("avoid-qt,Q", "debug: don't call Qt at all (implies --no-gui and --no-threads; attempts to be even more deterministic than normal)")
      ("no-frame-timing", bool_switch_off(&config.show_frame_timing), "don't log timing info each frame")
      ("no-threads", "debug: don't use threads even when supported")
      ("no-sim-thread", bool_switch_off(&config.use_simulation_thread), "debug: don't use a thread for the simulation")
      ("no-opengl-thread", bool_switch_off(&config.use_opengl_thread), "debug: use a thread for the OpenGL calls")
#if !LASERCAKE_NO_SELF_TESTS
      ("run-self-tests", "alternate run mode: run Lasercake's self-tests")
#endif
    ;

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).
              options(desc).positional(p).run(), vm);
    po::notify(vm);

    config.view_radius =
      physical_quantity<lint64_t, tile_widths_t>(
          vm["view-radius"].as<uint32_t>() * tile_widths)
      * identity(fine_distance_units / tile_widths);
    if(!config.run_drawing_code) {
      config.have_gui = false;
    }

    if(vm.count("no-threads")) {
      config.use_simulation_thread = false;
      config.use_opengl_thread = false;
    }
    if(vm.count("help")) {
      std::cout << desc << std::endl;
      std::cout << "Scenario names:";
      for(auto const& name : scenario_names()) {
        std::cout << "\n  " << name;
      }
      std::cout << std::endl;
      exit(0);
    }
#if !LASERCAKE_NO_SELF_TESTS
    if(vm.count("run-self-tests")) {
      return lasercake_test_main(argc, argv);
    }
#endif
    if(!vm.count("scenario")) {
      LOG << "You didn't give an argument saying which scenario to use! Using default value...\n";
      config.scenario = "default";
    }

    if(vm.count("avoid-qt")) {
      config.have_gui = false;
      return debug_test_sim_avoiding_qt_and_trying_to_be_very_deterministic (config);
    }
  }

  QCoreApplication::setAttribute(Qt::AA_X11InitThreads);
  QApplication qapp(argc, argv);
  if(config.have_gui) {
    if (!QGLFormat::hasOpenGL()) {
      LOG << "OpenGL capabilities not found; giving up." << std::endl;
      exit(1);
    }
  }
  qRegisterMetaType<worldgen_function_t>("worldgen_function_t");
  qRegisterMetaType<input_news_t>("input_news_t");
  qRegisterMetaType<frame_output_t>("input_representation::key_change_t");
  qRegisterMetaType<frame_output_t>("frame_output_t");
  qRegisterMetaType<gl_data_ptr_t>("gl_data_ptr_t");
  qRegisterMetaType<config_struct>("config_struct");
  qRegisterMetaType<distance>("distance");
  qRegisterMetaType<time_unit>("time_unit");

  QFontDatabase::addApplicationFont(":/resources/VC_Granger_ch8plus.ttf");
  QFontDatabase::addApplicationFont(":/resources/VC_Luna.ttf");
  QFontDatabase::addApplicationFont(":/resources/VC_Slytherin.ttf");
  QFontDatabase::addApplicationFont(":/resources/VC_Gryffindor.ttf");

  LasercakeController controller(config);
  // We call exit() here rather than return so that 'controller' is not
  // destroyed in the process of exiting the program.
  // This makes Qt emit fewer warnings; probably makes quitting a bit faster;
  // and we make sure not to use the destructor for any critical
  // end-of-process stuff (if there even is any currently).
  const int exitcode = qapp.exec();
  LOG << "Qt main loop has ended; calling exit()." << std::endl;
  exit(exitcode);
}

microseconds_t LasercakeGLThread::gl_render(gl_data_ptr_t& gl_data_ptr, LasercakeGLWidget& gl_widget, QSize viewport_size) {
  gl_widget.makeCurrent();
  BOOST_SCOPE_EXIT((&gl_widget)) {
   gl_widget.doneCurrent();
  } BOOST_SCOPE_EXIT_END
  // (should we measure time just here or include more of the loop?)
  // (Is monotonic appropriate?  Consider GL semantics: the implementation is
  // free to fork more threads, and/or wait for the GPU without consuming CPU,
  // etc [mine seems to do both somewhat].)
  const microseconds_t microseconds_before_gl = get_monotonic_microseconds();
  gl_renderer_.output_gl_data_to_OpenGL(
    *gl_data_ptr,
    viewport_size.width(),
    viewport_size.height(),
    gl_widget,
    gl_thread_data_->interrupt
  );
  if(!gl_thread_data_->interrupt.load()) {
    //TODO measure the microseconds ~here~ in the different configurations. e.g. should the before/after here be split across threads?
    //gl_data_ptr.reset(); // but if the deletion does happen now, it'll be in this thread, now, delaying swapBuffers etc :(
    // but it's a good time and CPU(cache) to delete the data on...
    gl_widget.swapBuffers();
  }
  const microseconds_t microseconds_after_gl = get_monotonic_microseconds();
  return microseconds_after_gl - microseconds_before_gl;
}

void LasercakeGLWidget::invoke_render_() {
  if(!has_quit_) {
    if(use_separate_gl_thread_) {
      gl_thread_data_->wait_for_instruction.wakeAll();
    }
    else {
      gl_thread_data_->microseconds_last_gl_render_took =
        thread_.gl_render(gl_thread_data_->current_data, *this, gl_thread_data_->viewport_size);
    }
  }
}
LasercakeGLWidget::LasercakeGLWidget(bool use_separate_gl_thread, QWidget* parent)
  : QGLWidget(parent),
    input_is_grabbed_(false),
    use_separate_gl_thread_(use_separate_gl_thread),
    has_quit_(false) {
  QObject::connect(
    QCoreApplication::instance(), SIGNAL(aboutToQuit()),
    this, SLOT(prepare_to_cleanly_close_()));
  setFocusPolicy(Qt::ClickFocus);
  //TODO setWindowFlags(), setWindow*()?
  //TODO do we want to request/check anything about the GL context?
  setWindowTitle("Lasercake");
  // default window size to 2/3 of available screen width and height
  const QRect approximateAvailableScreenArea = QApplication::desktop()->availableGeometry();
  resize(approximateAvailableScreenArea.width()*2/3, approximateAvailableScreenArea.height()*2/3);
  setAutoBufferSwap(false);
  setAutoFillBackground(false);
  gl_thread_data_.reset(new gl_thread_data_t);
  gl_thread_data_->current_data.reset(new gl_data_t());
  gl_thread_data_->viewport_size = size();
  gl_thread_data_->microseconds_last_gl_render_took = 0;
  gl_thread_data_->quit_now = false;
  gl_thread_data_->interrupt.store(false);
  gl_thread_data_->revision = 0;
  ++gl_thread_data_->revision; //display right away!
  thread_.gl_widget_ = this;
  thread_.gl_thread_data_ = gl_thread_data_;
  if(use_separate_gl_thread_) {
    thread_.last_revision_ = 0;
    thread_.microseconds_this_gl_render_took_ = 0;
    thread_.start();
  }
  else {
    gl_thread_data_->microseconds_last_gl_render_took =
      thread_.gl_render(gl_thread_data_->current_data, *this, size());
  }
}
void LasercakeGLWidget::update_gl_data(gl_data_ptr_t data) {
  {
    QMutexLocker lock(&gl_thread_data_->gl_data_lock);
    gl_thread_data_->current_data = data;
    ++gl_thread_data_->revision;
  }
  invoke_render_();
}
microseconds_t LasercakeGLWidget::get_last_gl_render_microseconds() {
  QMutexLocker lock(&gl_thread_data_->gl_data_lock);
  return gl_thread_data_->microseconds_last_gl_render_took;
}
void LasercakeGLThread::run() {
  assert(gl_thread_data_->interrupt.load() == false);
  while(true) {
    gl_data_ptr_t gl_data_ptr;
    QSize viewport_size;
    {
      QMutexLocker lock(&gl_thread_data_->gl_data_lock);
      gl_thread_data_->microseconds_last_gl_render_took = microseconds_this_gl_render_took_;
      if(last_revision_ == gl_thread_data_->revision) {
        gl_thread_data_->wait_for_instruction.wait(&gl_thread_data_->gl_data_lock);
      }
      if(gl_thread_data_->quit_now) {
        LOG << "Releasing OpenGL state... " << std::flush;
        gl_renderer_.fini();
        LOG << "Done releasing OpenGL state. " << std::flush;
        return;
      }
      gl_data_ptr = gl_thread_data_->current_data;
      last_revision_ = gl_thread_data_->revision;
      viewport_size = gl_thread_data_->viewport_size;
    }
    microseconds_this_gl_render_took_ =
      this->gl_render(gl_data_ptr, *gl_widget_, viewport_size);
  }
}
void LasercakeGLWidget::resizeEvent(QResizeEvent*) {
  QMutexLocker lock(&gl_thread_data_->gl_data_lock);
  gl_thread_data_->viewport_size = size();
}
void LasercakeGLWidget::paintEvent(QPaintEvent*) {
  {
    QMutexLocker lock(&gl_thread_data_->gl_data_lock);
    ++gl_thread_data_->revision;
  }
  invoke_render_();
}
void LasercakeGLWidget::prepare_to_cleanly_close_() {
  if(!has_quit_) {
    // Send debug messages in case waiting for any of these
    // operations freezes, so that people can tell exactly what
    // happened.

    // Just in case closing doesn't cause the OS to
    // release a mouse grab, we do so explicitly.
    if(input_is_grabbed_) {
      LOG << "Ungrabbing input... " << std::flush;
      ungrab_input_();
      LOG << "Done ungrabbing input. " << std::endl;
    }

    // Some OpenGL implementations don't like being
    // silently terminated, because they are poorly tested
    // piles of looming kernel-exploitation vulnerabilities.
    //
    // Anyway, waiting for the GL thread to reach a stopping point
    // seems to be useful for system stability.

    // TODO should we catch signals like Ctrl-C
    // in order to clean up even in their wake?
    LOG << (use_separate_gl_thread_ ? "Quitting OpenGL thread... "
                                    : "Releasing OpenGL state... ") << std::flush;
    {
      QMutexLocker lock(&gl_thread_data_->gl_data_lock);
      gl_thread_data_->quit_now = true;
      ++gl_thread_data_->revision;
    }
    if(use_separate_gl_thread_) {
      gl_thread_data_->interrupt.store(true);
      gl_thread_data_->wait_for_instruction.wakeAll();
      thread_.wait();
    }
    else {
      thread_.gl_renderer_.fini();
    }
    LOG << (use_separate_gl_thread_ ? "Done quitting OpenGL thread. "
                                    : "Done releasing OpenGL state. ") << std::flush;

    LOG << std::endl;
    has_quit_ = true;
  }
}
void LasercakeGLWidget::closeEvent(QCloseEvent* event) {
  prepare_to_cleanly_close_();
  QGLWidget::closeEvent(event);
}

// Don't let QEvent::event() steal tab keys or such, throwing
// off our counts of which keys are pressed.  If we want such
// behaviour in this widget we shall implement it ourself.
bool LasercakeGLWidget::event(QEvent* event) {
  switch (event->type()) {
    case QEvent::KeyPress:
      key_change_((QKeyEvent*)event, true);
      return true;
    case QEvent::KeyRelease:
      key_change_((QKeyEvent*)event, false);
      return true;
    default:
      return QGLWidget::event(event);
  }
}

namespace /*anonymous*/ {
//hack for using Qt non-public API:
//(http://kunalmaemo.blogspot.com/2010/05/enum-value-to-string-in-qt.html)
struct StaticQtMetaObject : QObject {
  static QMetaObject const& get() {return staticQtMetaObject;}
};

// This implementation semantic is probably a hack, but will do for now.
// TODO if the wire or especially the user comes to depend
// on these names, think more seriously about them.
//  Also consider the e.g. sequence on a US keyboard
//      PRESS Shift
//      PRESS /?     --> text ?, key /
//      RELEASE Shift
//      RELEASE /?   --> text /, key /
// So for single-key type things, we can't easily rely on
// event->text() at all.
// input_representation::key_type could *be* Qt::Key reasonably.
// There probably also needs to be a text entry *mode*...
// and I don't know enough about what it's like for e.g. Chinese keyboards.
input_representation::key_type q_key_event_to_input_rep_key_type(QKeyEvent* event) {
  QString as_text;// = event->text();
  //if(as_text.isEmpty() || QRegExp("[[:graph:]].*").exactMatch(as_text)) {
    QMetaObject const& staticQtMetaObject = StaticQtMetaObject::get();
    const int key_index = staticQtMetaObject.indexOfEnumerator("Key");
    const QMetaEnum key_meta_enum = staticQtMetaObject.enumerator(key_index);
    QString keyString = key_meta_enum.valueToKey(event->key());
    keyString.replace("Key_","");
    as_text = keyString;
  //}
  QString lowercase_text = QApplication::keyboardInputLocale().toLower(as_text);
  input_representation::key_type result(lowercase_text.toUtf8().constData());
  return result;
}
}

void LasercakeGLWidget::toggle_fullscreen() {
  setWindowState(windowState() ^ Qt::WindowFullScreen);
}
void LasercakeGLWidget::toggle_fullscreen(bool fullscreen) {
  if(fullscreen) { setWindowState(windowState() |  Qt::WindowFullScreen); }
  else           { setWindowState(windowState() & ~Qt::WindowFullScreen); }
}

namespace {
input_representation::key_type mouse_button_to_input_rep(Qt::MouseButton button) {
  switch (button) {
    case Qt::NoButton: caller_error("invalidly pressed/released Qt::NoButton");
    case Qt::LeftButton: return input_representation::left_mouse_button;
    case Qt::RightButton: return input_representation::right_mouse_button;
    case Qt::MiddleButton: return input_representation::middle_mouse_button;
    // Are these the correct numbers?
    case Qt::XButton1: return input_representation::mouse_button_n(4);
    case Qt::XButton2: return input_representation::mouse_button_n(5);
    default:
      LOG << "Unknown mouse button that Qt calls " << button << "\n";
      // give it some name anyhow, even if it's the same name
      // for all unknown buttons
      return input_representation::mouse_button_n(6);
  }
}
}
void LasercakeGLWidget::mousePressEvent(QMouseEvent* event) {
  if(!input_is_grabbed_) {
    global_cursor_pos_ = event->globalPos();
    grab_input_();
  }
  else {
    key_change_(-event->button(), mouse_button_to_input_rep(event->button()), true);
  }
  QGLWidget::mousePressEvent(event);
}
void LasercakeGLWidget::mouseReleaseEvent(QMouseEvent* event) {
  key_change_(-event->button(), mouse_button_to_input_rep(event->button()), false);
  QGLWidget::mouseReleaseEvent(event);
}

void LasercakeGLWidget::grab_input_() {
  if(!has_quit_) {
    input_is_grabbed_ = true;
    setMouseTracking(true);
    grabMouse();
    setCursor(QCursor(Qt::BlankCursor));
  }
}
void LasercakeGLWidget::ungrab_input_() {
  if(input_is_grabbed_) {
    setMouseTracking(false);
    releaseMouse();
    input_is_grabbed_ = false;
    setCursor(QCursor(Qt::ArrowCursor));
  }
}

void LasercakeGLWidget::mouseMoveEvent(QMouseEvent* event) {
  const QPoint new_global_cursor_pos = event->globalPos();
  const QPoint displacement = new_global_cursor_pos - global_cursor_pos_;
  QApplication::desktop()->cursor().setPos(global_cursor_pos_);
  input_rep_mouse_displacement_ += input_representation::mouse_displacement_t(
    displacement.x(), -displacement.y());
}
void LasercakeGLWidget::key_change_(
      qt_key_type_ qkey,
      input_representation::key_type ikey,
      bool pressed) {
  const input_representation::key_change_t input_rep_key_change(
    ikey, (pressed ? input_representation::PRESSED : input_representation::RELEASED));
  if(pressed) {
    // Have the input_representation believe there's only one of each key,
    // for its sanity.
    if(keys_currently_pressed_.find(qkey) == keys_currently_pressed_.end()) {
      //TODO use the Qt key enumeration in input_representation too
      //as it's much more complete than anything we'll ever have.
      input_rep_key_activity_.push_back(input_rep_key_change);
      input_rep_keys_currently_pressed_.insert(ikey);
      Q_EMIT key_changed(input_rep_key_change);
    }
    keys_currently_pressed_.insert(qkey);
  }
  else {
    const auto iter = keys_currently_pressed_.find(qkey);
    if(iter == keys_currently_pressed_.end()) {
      // This could happen if they focus this window while holding
      // a key down, and it not be a bug, so it's a bit silly to
      // log this.
      //LOG << "Key released but never pressed: " << qkey << " (" << q_key_event_to_input_rep_key_type(event) << ")\n";
    }
    else {
      keys_currently_pressed_.erase(iter);
      if(keys_currently_pressed_.find(qkey) == keys_currently_pressed_.end()) {
        input_rep_key_activity_.push_back(input_rep_key_change);
        input_rep_keys_currently_pressed_.erase(ikey);
        Q_EMIT key_changed(input_rep_key_change);
      }
    }
  }
}

void LasercakeGLWidget::key_change_(QKeyEvent* event, bool pressed) {
  if(event->isAutoRepeat()) {
    // If we someday allow key compression, this won't be a sensible way to stop auto-repeat:
    // "Note that if the event is a multiple-key compressed event that is partly due to auto-repeat, this function could return either true or false indeterminately."
    return;
  }
  if(pressed) {
    //TODO why handle window things here but other things in LasercakeController::key_changed?
    switch(event->key()) {
      case Qt::Key_Escape:
        ungrab_input_();
        return;
      case Qt::Key_Q:
      case Qt::Key_W:
        if((event->modifiers() & Qt::ControlModifier)) {
          close();
          return;
        }
        break;
      case Qt::Key_F11:
        toggle_fullscreen();
        return;
      case Qt::Key_F:
        // F11 does Exposé stuff in OS X, and Command-Shift-F seems to be
        // the "fullscreen" convention there.
        // (Qt maps OSX command-key to Qt::Key_Control.)
        if((event->modifiers() & Qt::ShiftModifier) && (event->modifiers() & Qt::ControlModifier)) {
          toggle_fullscreen();
          return;
        }
        break;
    }
  }
  const input_representation::key_type input_rep_key = q_key_event_to_input_rep_key_type(event);
  key_change_(event->key(), input_rep_key, pressed);
}

void LasercakeGLWidget::focusOutEvent(QFocusEvent*) {
  for(auto key : input_rep_keys_currently_pressed_) {
    const input_representation::key_change_t key_change(key, input_representation::RELEASED);
    input_rep_key_activity_.push_back(key_change);
    Q_EMIT key_changed(key_change);
  }
  input_rep_keys_currently_pressed_.clear();
  keys_currently_pressed_.clear();
}

input_representation::input_news_t LasercakeGLWidget::get_input_news()const {
  input_representation::input_news_t result(
    input_rep_keys_currently_pressed_, input_rep_key_activity_, input_rep_mouse_displacement_);
  return result;
}
void LasercakeGLWidget::clear_input_news() {
  input_rep_key_activity_.clear();
  input_rep_mouse_displacement_ = input_representation::mouse_displacement_t();
}



void LasercakeController::key_changed(input_representation::key_change_t key_change) {
  // Handle things that shouldn't wait until the next time (if any)
  // that the simulation thread gives us data.
  if(key_change.second == input_representation::PRESSED) {
    input_representation::key_type const& k = key_change.first;
    if(k == "p") {
      paused_ = !paused_;
      steps_queued_to_do_while_paused_ = 0;
      if(config_.have_gui) {
        gl_widget_->clear_input_news();
      }
      if(!paused_) {
        invoke_simulation_step_();
      }
    }
    if(k == "g") {
      if(paused_) {
        ++steps_queued_to_do_while_paused_;
        if(steps_queued_to_do_while_paused_ == 1) {
          invoke_simulation_step_();
        }
      }
    }
  }
}

LasercakeController::LasercakeController(config_struct config, QObject* parent)
 : QObject(parent), config_(config), frame_(0), game_time_(0), paused_(false), steps_queued_to_do_while_paused_(0)
{
  const worldgen_function_t worldgen = make_world_building_func(config_.scenario);
  if(!worldgen) {
    LOG << "Scenario name given that doesn't exist!: \'" << config_.scenario << "\'\n";
    // This constructor is called outside of QCoreApplication::exec(),
    // so QCoreApplication::exit() would be meaningless here
    // (at least I think that's why QCoreApplication::exit() didn't work).
    ::exit(4);
  }

  if(config_.have_gui) {
    gl_widget_.reset(new LasercakeGLWidget(config_.use_opengl_thread));
    QObject::connect(&*gl_widget_, SIGNAL(key_changed(input_representation::key_change_t)),
                    this,         SLOT(key_changed(input_representation::key_change_t)));
    gl_widget_->show();
  }

  simulator_.reset(new LasercakeSimulator());
  QObject::connect(&*simulator_, SIGNAL(frame_output_ready(time_unit, frame_output_t)),
                   this,         SLOT(output_new_frame  (time_unit, frame_output_t)),
                   Qt::AutoConnection);
  if(config_.use_simulation_thread) {
    simulator_thread_.reset(new QThread());
    simulator_->moveToThread(&*simulator_thread_);
    simulator_thread_->start();
  }

  monotonic_microseconds_at_beginning_of_frame_ = get_monotonic_microseconds();
  monotonic_microseconds_at_beginning_of_ten_frame_block_ = monotonic_microseconds_at_beginning_of_frame_;
  monotonic_microseconds_at_beginning_of_hundred_frame_block_ = monotonic_microseconds_at_beginning_of_frame_;

  // Initialization of the simulation has to happen in its thread for several
  // reasons:
  // Latency, in case there's a bug or slowness in the sim init code.
  // Future security, in case we make the sim thread into a sandbox someday.
  // (Minor) cache reasons, so that the newly created world will be in the
  //   relevant CPU's cache.
  QMetaObject::invokeMethod(&*simulator_, "init", Qt::AutoConnection,
    Q_ARG(worldgen_function_t, worldgen), Q_ARG(config_struct, config_));

  QMetaObject::invokeMethod(&*simulator_, "prepare_graphics", Qt::AutoConnection,
    Q_ARG(input_news_t, input_news_t()), Q_ARG(distance, config_.view_radius), Q_ARG(bool, config_.run_drawing_code));
}

// returns false if exiting the program
bool LasercakeController::invoke_simulation_step_() {
  ++frame_;
  if(config_.exit_after_frames == frame_) {
    QCoreApplication::quit();
    return false;
  }

  // get the simulator going again as promptly as possible
  const bool paused_this_time = paused_ && steps_queued_to_do_while_paused_ == 0;
  if(steps_queued_to_do_while_paused_ > 0) {
    --steps_queued_to_do_while_paused_;
  }
  if(!paused_this_time) {
    input_representation::input_news_t input_news;
    if(config_.have_gui) {
      input_news = gl_widget_->get_input_news();
      gl_widget_->clear_input_news();
    }

    QMetaObject::invokeMethod(&*simulator_, "new_input_as_of", Qt::QueuedConnection,
      Q_ARG(time_unit, game_time_), Q_ARG(input_news_t, input_news));
    QMetaObject::invokeMethod(&*simulator_, "prepare_graphics", Qt::QueuedConnection,
      Q_ARG(input_news_t, input_news), Q_ARG(distance, config_.view_radius), Q_ARG(bool, config_.run_drawing_code));
  }
  return true;
}

void LasercakeController::output_new_frame(time_unit moment, frame_output_t output) {
  game_time_ = moment;
  if(!invoke_simulation_step_()) {return;}

  microseconds_t last_gl_render_microseconds = 0;
  if(config_.have_gui) {
    // get() before because update_gl_data may be asynchronous,
    // and it's nice to be consistent about when this value refers to.
    last_gl_render_microseconds = gl_widget_->get_last_gl_render_microseconds();
    gl_widget_->update_gl_data(output.gl_data_ptr);
  }

  const microseconds_t end_frame_monotonic_microseconds = get_monotonic_microseconds();
  const microseconds_t monotonic_microseconds_for_frame = end_frame_monotonic_microseconds - monotonic_microseconds_at_beginning_of_frame_;

  if(config_.show_frame_timing) {
    LOG << output.extra_debug_info;
  }

  if(config_.show_frame_timing) { LOG
  << "Frame " << std::left << std::setw(4) << frame_ << std::right << ":"
  //TODO bugreport: with fps as double, this produced incorrect results for me-- like multiplying output by 10
  // -- may be a libstdc++ bug (or maybe possibly me misunderstanding the library)
  //<< std::ios::fixed << std::setprecision(4) << fps << " fps; "
  << std::setw(4) << get_this_process_mem_usage_megabytes() << "MiB; "
  << std::setw(6) << show_microseconds_per_frame_as_fps(monotonic_microseconds_for_frame) << "fps"
  << std::setw(6) << show_microseconds(monotonic_microseconds_for_frame) << "ms"
  << ":"
  << std::setw(7) << show_microseconds(output.microseconds_for_simulation) << "sim"
  << std::setw(6) << show_microseconds(output.microseconds_for_drawing) << "prep"
  << ":" << std::setw(6) << show_microseconds(output.microseconds_for_simulation + output.microseconds_for_drawing) << "s+p"
  << "  (" << show_microseconds(last_gl_render_microseconds) << "ms last gl)"
  << "\n";
  }

  if(frame_ % 10 == 0) {
    const microseconds_t beginning = monotonic_microseconds_at_beginning_of_ten_frame_block_;
    monotonic_microseconds_at_beginning_of_ten_frame_block_ = end_frame_monotonic_microseconds;
    const microseconds_t ending = monotonic_microseconds_at_beginning_of_ten_frame_block_;

    if(config_.show_frame_timing) { LOG
    << show_microseconds_per_frame_as_fps((ending - beginning) / 10)
    << " fps over the last ten frames " << (frame_-10) << "-" << frame_ << ".\n";
    }
  }
  if(frame_ % 100 == 0) {
    const microseconds_t beginning = monotonic_microseconds_at_beginning_of_hundred_frame_block_;
    monotonic_microseconds_at_beginning_of_hundred_frame_block_ = end_frame_monotonic_microseconds;
    const microseconds_t ending = monotonic_microseconds_at_beginning_of_hundred_frame_block_;

    if(config_.show_frame_timing) { LOG
    << show_microseconds_per_frame_as_fps((ending - beginning) / 100)
    << " fps over the last hundred frames " << (frame_-100) << "-" << frame_ << ".\n";
    }
  }

  // HACK - limit to about 60 FPS
  // TODO - use Qt timer events instead.
  // TODO - make this configurable and/or changeable
  // at runtime.
  const microseconds_t frame_duration_at_60fps = 1000000 / 60;
  const microseconds_t estimated_extra_delay_after_sleep = frame_duration_at_60fps / 6;
  const microseconds_t want_to_sleep =
    frame_duration_at_60fps - last_gl_render_microseconds - estimated_extra_delay_after_sleep;
  if(want_to_sleep > 0) {
    sleeper::usleep(want_to_sleep);
  }

  monotonic_microseconds_at_beginning_of_frame_ = end_frame_monotonic_microseconds;
}

