#ifndef NUTILS_H
#define NUTILS_H

void topicadvertisedTip(const char *topic);

#define ARRAY_ASSIGN(DEST, SRC)                                                \
  for (size_t _CNT = 0; _CNT < sizeof(SRC) / sizeof(SRC[0]); ++_CNT) {         \
    DEST[_CNT] = SRC[_CNT];                                                    \
  }
  
#endif // NUTILS_H
