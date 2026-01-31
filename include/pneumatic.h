#ifndef PNEUMATIC_H
#define PNEUMATIC_H

#include "vex.h"

using namespace vex;

class PneumaticController {
public:
  PneumaticController(digital_out& pneumatic)
      : m_pneumatic(pneumatic), m_isExtended(false) {
    m_pneumatic.set(false);
  }

  void toggle() {
    m_isExtended = !m_isExtended;
    m_pneumatic.set(m_isExtended);
  }

  bool isExtended() const { return m_isExtended; }

  const char* getStatusString() const {
    return m_isExtended ? "UP" : "DOWN";
  }

private:
  digital_out& m_pneumatic;
  bool m_isExtended;
};

#endif
