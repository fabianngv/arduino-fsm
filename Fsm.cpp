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

#include "Fsm.h"

#if !defined(ARDUINO)
#include <sys/time.h>
unsigned long millis() {
  struct timeval tp;
  gettimeofday(&tp, nullptr);
  return tp.tv_sec * 1000 + tp.tv_usec / 1000;
}
#endif

State::State(void (*on_enter)(), void (*on_state)(), void (*on_exit)())
    : StateInterface(), on_enter(on_enter), on_state(on_state),
      on_exit(on_exit) {}

StateMember::StateMember(FsmMemFn on_enter, FsmMemFn on_state, FsmMemFn on_exit,
                         Fsm *fsm)
    : StateInterface(), on_enter(on_enter), on_state(on_state),
      on_exit(on_exit), fsm(fsm) {}

Fsm::Fsm(StateInterface *initial_state)
    : m_current_state(initial_state), m_transitions(nullptr),
      m_timed_transitions(nullptr), m_initialized(false) {}

Fsm::~Fsm() {
  TransitionInterface *t = m_transitions;
  while (t != nullptr) {
    TransitionInterface *transition = t;
    t = transition->next;
    delete transition;
  }
  TimedTransition *tt = m_timed_transitions;
  while (tt != nullptr) {
    TimedTransition *timed_transition = tt;
    tt = timed_transition->next;
    delete timed_transition->transition;
    delete timed_transition;
  }
  m_transitions = nullptr;
  m_timed_transitions = nullptr;
}

void Fsm::add_transition(StateInterface *state_from, StateInterface *state_to,
                         int event, void (*on_transition)()) {
  add_transition(create_transition(state_from, state_to, event, on_transition));
}

void Fsm::add_transition(StateInterface *state_from, StateInterface *state_to,
                         int event, FsmMemFn on_transition, Fsm *fsm) {

  add_transition(
      create_transition(state_from, state_to, event, on_transition, fsm));
}

void Fsm::add_transition(TransitionInterface *transition) {
  if (transition == nullptr)
    return;

  if (m_transitions != nullptr) {
    TransitionInterface *head = m_transitions;
    while (head->next != nullptr)
      head = head->next;
    head->next = transition;
  } else
    m_transitions = transition;
}

void Fsm::add_timed_transition(StateInterface *state_from,
                               StateInterface *state_to, unsigned long interval,
                               void (*on_transition)()) {
  add_timed_transition(
      interval, create_transition(state_from, state_to, 0, on_transition));
}

void Fsm::add_timed_transition(StateInterface *state_from,
                               StateInterface *state_to, unsigned long interval,
                               FsmMemFn on_transition, Fsm *fsm) {
  add_timed_transition(
      interval, create_transition(state_from, state_to, 0, on_transition, fsm));
}

void Fsm::add_timed_transition(unsigned long interval,
                               TransitionInterface *transition) {
  if (transition == nullptr)
    return;

  TimedTransition *timed_transition = new TimedTransition();
  timed_transition->transition = transition;
  timed_transition->start = 0;
  timed_transition->interval = interval;
  timed_transition->next = nullptr;

  if (m_timed_transitions != nullptr) {
    TimedTransition *head = m_timed_transitions;
    while (head->next != nullptr)
      head = head->next;
    head->next = timed_transition;
  } else
    m_timed_transitions = timed_transition;
}

Fsm::TransitionInterface *Fsm::create_transition(StateInterface *state_from,
                                                 StateInterface *state_to,
                                                 int event,
                                                 void (*on_transition)()) {

  Transition *transition = new Transition();
  transition->on_transition = on_transition;

  return create_transition(state_from, state_to, event, transition);
}

Fsm::TransitionInterface *
Fsm::create_transition(StateInterface *state_from, StateInterface *state_to,
                       int event, FsmMemFn on_transition, Fsm *fsm) {

  TransitionMember *transition = new TransitionMember();
  transition->on_transition = on_transition;
  transition->fsm = fsm;

  return create_transition(state_from, state_to, event, transition);
}

Fsm::TransitionInterface *
Fsm::create_transition(StateInterface *state_from, StateInterface *state_to,
                       int event, TransitionInterface *transition) {
  if (state_from == nullptr || state_to == nullptr) {
    delete transition;
    return nullptr;
  }

  transition->state_from = state_from;
  transition->state_to = state_to;
  transition->event = event;
  transition->next = nullptr;

  return transition;
}

void Fsm::trigger(int event) {
  if (m_initialized) {
    // Find the transition with the current state and given event.
    TransitionInterface *head = m_transitions;
    while (head != nullptr) {
      TransitionInterface *transition = head;
      if (transition->state_from == m_current_state &&
          transition->event == event) {
        Fsm::make_transition(transition);
        return;
      }
      head = transition->next;
    }
  }
}

void Fsm::check_timed_transitions() {
  TimedTransition *head = m_timed_transitions;
  while (head != nullptr) {
    TimedTransition *ttransition = head;
    if (ttransition->transition->state_from == m_current_state) {
      if (ttransition->start == 0) {
        ttransition->start = millis();
      } else {
        unsigned long now = millis();
        if (now - ttransition->start >= ttransition->interval) {
          Fsm::make_transition(ttransition->transition);
          ttransition->start = 0;
        }
      }
    }
    head = ttransition->next;
  }
}

void Fsm::run_machine() {
  // first run must exec first state "on_enter"
  if (!m_initialized) {
    m_initialized = true;
    m_current_state->enter();
  }

  m_current_state->state();

  Fsm::check_timed_transitions();
}

StateInterface *Fsm::get_current_state() { return m_current_state; }

void Fsm::make_transition(TransitionInterface *transition) {
  // Execute the handlers in the correct order.
  transition->state_from->exit();

  transition->transition();

  transition->state_to->enter();

  m_current_state = transition->state_to;

  // Initialice all timed transitions from m_current_state
  reset_timers();
}

void Fsm::reset_timers(void) {
  // Initialice all timed transitions from m_current_state
  unsigned long now = millis();
  TimedTransition *head = m_timed_transitions;
  while (head != nullptr) {
    TimedTransition *ttransition = head;
    if (ttransition->transition->state_from == m_current_state)
      ttransition->start = now;
    head = ttransition->next;
  }
}
