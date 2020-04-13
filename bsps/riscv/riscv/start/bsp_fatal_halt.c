/*
 * Copyright (c) 2018 embedded brains GmbH
 *
 * Copyright (c) 2015 University of York.
 * Hesham Almatary <hesham@alumni.york.ac.uk>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <bsp/riscv.h>
#include <bsp/fdt.h>

#include <libfdt.h>

#if __CHERI__
#include <rtems/score/cheri-utility.h>
#endif

void _CPU_Fatal_halt(uint32_t source, uint32_t error)
{
  const char *fdt;
  int node;
  volatile size_t *sifive_test;

#if RISCV_ENABLE_HTIF_SUPPORT != 0
  htif_poweroff();
#endif

  fdt = bsp_fdt_get();
  node = fdt_node_offset_by_compatible(fdt, -1, "sifive,test0");
  sifive_test = riscv_fdt_get_address(fdt, node);

#if __CHERI__
  sifive_test = cheri_build_data_cap((size_t) sifive_test, sizeof(size_t), 0xff);
#endif /* __CHERI__ */

  while (true) {
    if (sifive_test != NULL) {
      *sifive_test = 0x5555;
    }
  }
}
