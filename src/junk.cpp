//
// Created by peter on 27/11/2024.
//

<<<<<<<<<<<<<<  ✨ Codeium Command ⭐ >>>>>>>>>>>>>>>>
/**
 * Pauses the execution of the program for a specified number of milliseconds.
 *
 * This function repeatedly calls the `yield()` function to allow other
 * threads or processes to execute while waiting for the delay to complete.
 * It uses the `millis()` function to measure time passed.
 *
 * @param ms The number of milliseconds to delay.
 */

<<<<<<<  95ae4ba2-02f8-4f59-af18-8d68838fcea3  >>>>>>>
void delay_ms(int ms){
  uint32_t now = millis();
  while(millis() - now < ms){
    yield();
  }
}

while {lfs.value() < 235} {
  delay_ms(1);
}
