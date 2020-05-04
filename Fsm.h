// This file is part of arduino-fsm.
//
// arduino-fsm is free software: you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version.
//
// arduino-fsm is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License
// for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with arduino-fsm.  If not, see <http://www.gnu.org/licenses/>.

#ifndef FSM_H
#define FSM_H

#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#else
#include <cstddef>
#include <stdio.h>
#endif

#define CALL_MEMBER_FN(object, ptrToMember) ((object)->*(ptrToMember))

class Fsm;

typedef void (Fsm::*FsmMemFn)();

struct StateInterface {
  virtual void enter() = 0;
  virtual void state() = 0;
  virtual void exit() = 0;
};

struct State : StateInterface {
  State(void (*on_enter)(), void (*on_state)(), void (*on_exit)());
  void (*on_enter)();
  void (*on_state)();
  void (*on_exit)();
  void enter() {
    if (on_enter != nullptr)
      on_enter();
  };
  void state() {
    if (on_state != nullptr)
      on_state();
  };
  void exit() {
    if (on_exit != nullptr)
      on_exit();
  };
};

struct StateMember : StateInterface {
  StateMember(FsmMemFn on_enter, FsmMemFn on_state, FsmMemFn on_exit, Fsm *fsm);
  FsmMemFn on_enter;
  FsmMemFn on_state;
  FsmMemFn on_exit;
  Fsm *fsm;
  void enter() {
    if (on_enter != nullptr)
      CALL_MEMBER_FN(fsm, on_enter)();
  };
  void state() {
    if (on_state != nullptr)
      CALL_MEMBER_FN(fsm, on_state)();
  };
  void exit() {
    if (on_exit != nullptr)
      CALL_MEMBER_FN(fsm, on_exit)();
  };
};

class Fsm {
public:
  Fsm(StateInterface *initial_state);
  virtual ~Fsm();

  void add_transition(StateInterface *state_from, StateInterface *state_to,
                      int event, void (*on_transition)());

  void add_transition(StateInterface *state_from, StateInterface *state_to,
                      int event, FsmMemFn on_transition, Fsm *fsm);

  void add_timed_transition(StateInterface *state_from,
                            StateInterface *state_to, unsigned long interval,
                            void (*on_transition)());

  void add_timed_transition(StateInterface *state_from,
                            StateInterface *state_to, unsigned long interval,
                            FsmMemFn on_transition, Fsm *fsm);

  void check_timed_transitions();

  virtual void trigger(int event);
  void run_machine();

  StateInterface *get_current_state();

protected:
  struct TransitionInterface {
    StateInterface *state_from;
    StateInterface *state_to;
    int event;
    virtual void transition() = 0;
    TransitionInterface *next;
  };
  struct Transition : TransitionInterface {
    void (*on_transition)();
    void transition() {
      if (on_transition != nullptr) {
        on_transition();
      }
    }
  };
  struct TransitionMember : TransitionInterface {
    FsmMemFn on_transition;
    Fsm *fsm;
    void transition() {
      if (on_transition != nullptr)
        CALL_MEMBER_FN(fsm, on_transition)();
    };
  };
  struct TimedTransition {
    TransitionInterface *transition;
    unsigned long start;
    unsigned long interval;
    TimedTransition *next;
  };

  void make_transition(TransitionInterface *transition);

  void reset_timers();

private:
  void add_transition(TransitionInterface *transition);

  void add_timed_transition(unsigned long interval,
                            TransitionInterface *transition);

  TransitionInterface *create_transition(StateInterface *state_from,
                                         StateInterface *state_to, int event,
                                         void (*on_transition)());

  TransitionInterface *create_transition(StateInterface *state_from,
                                         StateInterface *state_to, int event,
                                         FsmMemFn on_transition, Fsm *fsm);

  TransitionInterface *create_transition(StateInterface *state_from,
                                         StateInterface *state_to, int event,
                                         TransitionInterface *transition);

  StateInterface *m_current_state;
  TransitionInterface *m_transitions;
  TimedTransition *m_timed_transitions;
  bool m_initialized;
};

#endif
