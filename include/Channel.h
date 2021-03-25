class Channel {
  public:
    Channel() {
      m_uStartT = 0;
      m_uEndT = 0;
      m_uElapsedT = 0;
    }

    uint32_t getElapsed() {
      uint32_t elapsed = m_uEndT - m_uStartT;
      // Filter out all the nonsense
      if (elapsed < 2100 && elapsed > 900) {
        // Increase elapsed time slowly to reduce noise and make behaviour less aggressive
        m_uElapsedT = (m_uElapsedT * 0.9f) + (0.1f * elapsed);
      }

      return m_uElapsedT;
    }

    inline int16_t getStickPosition() {
      return constrain((int16_t)(1500 - getElapsed()), -500, 500);
    }

    inline void setStartTime(uint32_t startT) {
      m_uStartT = startT;
    }

    inline void setEndTime(uint32_t endT) {
      m_uEndT = endT;
    }

  private:
    uint32_t m_uStartT;
    uint32_t m_uEndT;
    uint32_t m_uElapsedT;
};
