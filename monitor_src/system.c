void SystemCoreClockUpdate (void);
void SystemCoreClockUpdate (void) {
}

void SystemInit (void);
void SystemInit (void) {
  /* Update SystemCoreClock variable */
  SystemCoreClockUpdate();
}
