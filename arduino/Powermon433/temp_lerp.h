struct tagTempItem {
  uint8_t val;
  int8_t t;
} temp_tab[] = {
  { 0, -49 },
  //{ 5, -45 },
  { 10, -42 },
  { 20, -22 },
  { 30, -7 },
  { 40, 5 },
  { 50, 16 },
  { 60, 25 },
  { 70, 34 },
  { 80, 42 },
  { 90, 49 },
  { 100, 57 },
  { 110, 64 },
  { 120, 71 },
  { 130, 78 },
  { 140, 86 },
  { 150, 94 },
  //{ 152, 96 },
  //{ 154, 97 },
  //{ 156, 99 },
  //{ 158, 101 },
  { 160, 102 },
  //{ 162, 104 },
  //{ 164, 106 },
  { 166, 108 },
  { 176, 118 },
  { 180, 121 },
  //{ 184, 126 },
  { 185, 127 }
  //{255, 127 }
};

#define TEMPTAB_CNT (sizeof(temp_tab)/sizeof(temp_tab[0]))

int8_t temp_lerp(uint8_t val)
{
  uint8_t i;
  for (i=0; i<TEMPTAB_CNT-1; ++i)
  {
    if (val <= temp_tab[i].val)
    {
      int8_t a = temp_tab[i].t;
      int8_t b = temp_tab[i+1].t;

      uint8_t x = temp_tab[i].val;
      uint8_t y = temp_tab[i+1].val;

      return a + ((b - a) * (int16_t)(val - x) / (y - x));
    }
  }
  return temp_tab[TEMPTAB_CNT-1].t;
}
