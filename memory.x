MEMORY
{
  /* */
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 32K
  /* */
  RAM_VECTOR_TABLE (rwx): ORIGIN = 0x20000000, LENGTH = 192
  /* */
  RAM (rwx) : ORIGIN = 0x200000C0, LENGTH = 6K - 192
}

_stack_start = ORIGIN(RAM) + LENGTH(RAM);
