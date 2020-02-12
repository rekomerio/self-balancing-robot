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
      if (elapsed < 2050 && elapsed > 950) {
        // Increase elapsed time slowly to reduce noise
        this->m_uElapsedT = (this->m_uElapsedT * 0.8) + (0.2 * elapsed);
      }

      return m_uElapsedT;
    }

    inline void setStartTime(uint32_t startT) { m_uStartT = startT; }
    inline void setEndTime(uint32_t endT) { m_uEndT = endT; }

  private:
    uint32_t m_uStartT;
    uint32_t m_uEndT;
    uint32_t m_uElapsedT;
};