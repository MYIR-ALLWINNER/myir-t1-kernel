	.arch armv7-a
	.fpu softvfp
	.eabi_attribute 20, 1	@ Tag_ABI_FP_denormal
	.eabi_attribute 21, 1	@ Tag_ABI_FP_exceptions
	.eabi_attribute 23, 3	@ Tag_ABI_FP_number_model
	.eabi_attribute 24, 1	@ Tag_ABI_align8_needed
	.eabi_attribute 25, 1	@ Tag_ABI_align8_preserved
	.eabi_attribute 26, 2	@ Tag_ABI_enum_size
	.eabi_attribute 30, 4	@ Tag_ABI_optimization_goals
	.eabi_attribute 34, 1	@ Tag_CPU_unaligned_access
	.eabi_attribute 18, 2	@ Tag_ABI_PCS_wchar_t
	.file	"sunxi_can_asm.c"
@ GNU C89 (Linaro GCC 5.3-2016.05) version 5.3.1 20160412 (arm-linux-gnueabi)
@	compiled by GNU C version 4.8.4, GMP version 6.0.0, MPFR version 3.1.3, MPC version 1.0.3
@ GGC heuristics: --param ggc-min-expand=100 --param ggc-min-heapsize=131072
@ options passed:  -nostdinc -I ./arch/arm/include
@ -I ./arch/arm/include/generated -I ./include -I ./arch/arm/include/uapi
@ -I ./arch/arm/include/generated/uapi -I ./include/uapi
@ -I ./include/generated/uapi -imultilib . -imultiarch arm-linux-gnueabi
@ -iprefix /home/wujiayi/T113_release/out/gcc-linaro-5.3.1-2016.05-x86_64_arm-linux-gnueabi/bin/../lib/gcc/arm-linux-gnueabi/5.3.1/
@ -isysroot /home/wujiayi/T113_release/out/gcc-linaro-5.3.1-2016.05-x86_64_arm-linux-gnueabi/bin/../arm-linux-gnueabi/libc
@ -D __KERNEL__ -D __LINUX_ARM_ARCH__=7 -U arm
@ -D KBUILD_BASENAME="sunxi_can_asm" -D KBUILD_MODNAME="sunxi_can_asm"
@ -isystem /home/wujiayi/T113_release/out/gcc-linaro-5.3.1-2016.05-x86_64_arm-linux-gnueabi/bin/../lib/gcc/arm-linux-gnueabi/5.3.1/include
@ -include ./include/linux/kconfig.h
@ -include ./include/linux/compiler_types.h
@ -MD drivers/net/can/.sunxi_can_asm.s.d drivers/net/can/sunxi_can_asm.c
@ -mlittle-endian -mabi=aapcs-linux -mfpu=vfp -marm -march=armv7-a
@ -mfloat-abi=soft -mtune=cortex-a9 -mtls-dialect=gnu
@ -auxbase-strip drivers/net/can/sunxi_can_asm.s -Os -Wall -Wundef
@ -Werror=strict-prototypes -Wno-trigraphs
@ -Werror=implicit-function-declaration -Werror=implicit-int
@ -Wno-format-security -Wframe-larger-than=2048
@ -Wno-unused-but-set-variable -Wdeclaration-after-statement -Wvla
@ -Wno-pointer-sign -Wno-array-bounds -Wno-maybe-uninitialized
@ -Werror=date-time -Werror=incompatible-pointer-types
@ -Werror=designated-init -std=gnu90 -fno-strict-aliasing -fno-common
@ -fshort-wchar -fno-PIE -fno-dwarf2-cfi-asm -fno-ipa-sra -funwind-tables
@ -fno-delete-null-pointer-checks -fstack-protector-strong
@ -fomit-frame-pointer -fno-strict-overflow -fno-merge-all-constants
@ -fmerge-constants -fstack-check=no -fconserve-stack -fverbose-asm
@ --param allow-store-data-races=0
@ options enabled:  -faggressive-loop-optimizations -falign-functions
@ -falign-jumps -falign-labels -falign-loops -fauto-inc-dec
@ -fbranch-count-reg -fcaller-saves -fchkp-check-incomplete-type
@ -fchkp-check-read -fchkp-check-write -fchkp-instrument-calls
@ -fchkp-narrow-bounds -fchkp-optimize -fchkp-store-bounds
@ -fchkp-use-static-bounds -fchkp-use-static-const-bounds
@ -fchkp-use-wrappers -fcombine-stack-adjustments -fcompare-elim
@ -fcprop-registers -fcrossjumping -fcse-follow-jumps -fdefer-pop
@ -fdevirtualize -fdevirtualize-speculatively -fearly-inlining
@ -feliminate-unused-debug-types -fexpensive-optimizations
@ -fforward-propagate -ffunction-cse -fgcse -fgcse-lm -fgnu-runtime
@ -fgnu-unique -fguess-branch-probability -fhoist-adjacent-loads -fident
@ -fif-conversion -fif-conversion2 -findirect-inlining -finline
@ -finline-atomics -finline-functions -finline-functions-called-once
@ -finline-small-functions -fipa-cp -fipa-cp-alignment -fipa-icf
@ -fipa-icf-functions -fipa-icf-variables -fipa-profile -fipa-pure-const
@ -fipa-ra -fipa-reference -fira-hoist-pressure -fira-share-save-slots
@ -fira-share-spill-slots -fisolate-erroneous-paths-dereference -fivopts
@ -fkeep-static-consts -fleading-underscore -flifetime-dse -flra-remat
@ -flto-odr-type-merging -fmath-errno -fmerge-constants
@ -fmerge-debug-strings -fmove-loop-invariants -fomit-frame-pointer
@ -foptimize-sibling-calls -fpartial-inlining -fpeephole -fpeephole2 -fplt
@ -fprefetch-loop-arrays -freg-struct-return -freorder-blocks
@ -freorder-functions -frerun-cse-after-loop
@ -fsched-critical-path-heuristic -fsched-dep-count-heuristic
@ -fsched-group-heuristic -fsched-interblock -fsched-last-insn-heuristic
@ -fsched-pressure -fsched-rank-heuristic -fsched-spec
@ -fsched-spec-insn-heuristic -fsched-stalled-insns-dep -fschedule-insns2
@ -fsection-anchors -fsemantic-interposition -fshow-column -fshrink-wrap
@ -fsigned-zeros -fsplit-ivs-in-unroller -fsplit-wide-types -fssa-phiopt
@ -fstack-protector-strong -fstdarg-opt -fstrict-volatile-bitfields
@ -fsync-libcalls -fthread-jumps -ftoplevel-reorder -ftrapping-math
@ -ftree-bit-ccp -ftree-builtin-call-dce -ftree-ccp -ftree-ch
@ -ftree-coalesce-vars -ftree-copy-prop -ftree-copyrename -ftree-cselim
@ -ftree-dce -ftree-dominator-opts -ftree-dse -ftree-forwprop -ftree-fre
@ -ftree-loop-if-convert -ftree-loop-im -ftree-loop-ivcanon
@ -ftree-loop-optimize -ftree-parallelize-loops= -ftree-phiprop -ftree-pre
@ -ftree-pta -ftree-reassoc -ftree-scev-cprop -ftree-sink -ftree-slsr
@ -ftree-sra -ftree-switch-conversion -ftree-tail-merge -ftree-ter
@ -ftree-vrp -funit-at-a-time -funwind-tables -fverbose-asm
@ -fzero-initialized-in-bss -marm -mglibc -mlittle-endian
@ -mpic-data-is-text-relative -msched-prolog -munaligned-access
@ -mvectorize-with-neon-quad

	.syntax divided
	.syntax unified
	.arm
	.syntax unified
	.text
	.align	2
	.global	can_asm_write_cmdreg
	.syntax unified
	.arm
	.type	can_asm_write_cmdreg, %function
can_asm_write_cmdreg:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, lr}	@
	.save {r4, r5, r6, r7, lr}
	mov	r3, #16	@ tmp196,
	.pad #28
	sub	sp, sp, #28	@,,
	mov	r7, r0	@ mod_reg_val, mod_reg_val
	mov	r6, r1	@ mod_reg_addr, mod_reg_addr
	str	r3, [sp, #16]	@ tmp196, mod_reg_val
	mov	r3, #9	@ tmp197,
	str	r3, [sp, #20]	@ tmp197, mod_reg_addr
	ldr	r2, [sp, #16]	@ D.65714, mod_reg_val
	ldr	r3, [sp, #20]	@ D.65714, mod_reg_addr
	cmp	r2, r3	@ D.65714, D.65714
	bge	.L2	@,
	ldr	r3, [sp, #16]	@ D.65714, mod_reg_val
	ldr	r4, .L23	@ tmp200,
	ldr	r2, [sp, #20]	@ D.65714, mod_reg_addr
	smull	r4, r5, r3, r4	@ tmp199, D.65714, tmp200
	sub	r3, r5, r3, asr #31	@ D.65714,, D.65714,
	add	r3, r3, r2	@ D.65714, D.65714, D.65714
	str	r3, [sp, #20]	@ D.65714, mod_reg_addr
.L2:
	ldr	r3, [sp, #20]	@ D.65714, mod_reg_addr
	cmp	r3, #3072	@ D.65714,
	beq	.L19	@,
	ldr	r2, [sp, #16]	@ D.65714, mod_reg_val
	ldr	r3, [sp, #20]	@ D.65714, mod_reg_addr
	cmp	r2, r3	@ D.65714, D.65714
	ldr	r3, [sp, #20]	@ D.65714, mod_reg_addr
	blt	.L4	@,
	str	r3, [sp, #16]	@ D.65714, mod_reg_val
	movw	r3, #819	@ tmp202,
	ldr	r2, [sp, #16]	@ D.65714, mod_reg_val
	cmp	r2, r3	@ D.65714, tmp202
	ldr	r3, [sp, #20]	@ D.65714, mod_reg_addr
	beq	.L4	@,
	cmp	r3, #3072	@ D.65714,
	ldr	r3, [sp, #16]	@ D.65714, mod_reg_val
	beq	.L4	@,
	cmp	r3, #21504	@ D.65714,
	ldreq	r3, [sp, #20]	@ D.65714, mod_reg_addr
	beq	.L4	@,
	ldr	r3, [sp, #16]	@ D.65714, mod_reg_val
	and	r3, r3, #3072	@ D.65714, D.65714,
	str	r3, [sp, #20]	@ D.65714, mod_reg_addr
.L19:
	ldr	r3, [sp, #16]	@ D.65714, mod_reg_val
.L4:
	.syntax divided
@ 445 "drivers/net/can/sunxi_can_asm.c" 1
	dsb st
@ 0 "" 2
	.arm
	.syntax unified
	bl	arm_heavy_mb	@
	.syntax divided
@ 95 "./arch/arm/include/asm/io.h" 1
	str r7, [r6, #4]	@ mod_reg_val, MEM[(volatile u32 *)_6]
@ 0 "" 2
	.arm
	.syntax unified
	mov	r3, #11	@ tmp203,
	str	r3, [sp, #8]	@ tmp203, mod_reg_val
	mov	r3, #8	@ tmp204,
	str	r3, [sp, #12]	@ tmp204, mod_reg_addr
	movw	r3, #819	@ tmp205,
	ldr	r2, [sp, #12]	@ D.65714, mod_reg_addr
	mul	r2, r3, r2	@ D.65714, tmp205, D.65714
	str	r2, [sp, #8]	@ D.65714, mod_reg_val
	ldr	r2, [sp, #12]	@ D.65714, mod_reg_addr
	mul	r2, r3, r2	@ D.65714, tmp205, D.65714
	str	r2, [sp, #8]	@ D.65714, mod_reg_val
	ldr	r2, [sp, #8]	@ D.65714, mod_reg_val
	cmp	r2, r3	@ D.65714, tmp205
	beq	.L21	@,
	ldr	r3, [sp, #8]	@ D.65714, mod_reg_val
	cmp	r3, #3072	@ D.65714,
	beq	.L21	@,
	ldr	r2, [sp, #8]	@ D.65714, mod_reg_val
	ldr	r3, [sp, #12]	@ D.65714, mod_reg_addr
	cmp	r2, r3	@ D.65714, D.65714
	bge	.L12	@,
.L21:
	ldr	r3, [sp, #12]	@ D.65714, mod_reg_addr
	b	.L10	@
.L12:
	ldr	r2, [sp, #8]	@ D.65714, mod_reg_val
	ldr	r3, [sp, #12]	@ D.65714, mod_reg_addr
	eor	r3, r3, r2	@ D.65714, D.65714, D.65714
	str	r3, [sp, #12]	@ D.65714, mod_reg_addr
	ldr	r2, [sp, #8]	@ D.65714, mod_reg_val
	ldr	r3, [sp, #12]	@ D.65714, mod_reg_addr
	eor	r3, r3, r2	@ D.65714, D.65714, D.65714
	str	r3, [sp, #12]	@ D.65714, mod_reg_addr
	ldr	r2, [sp, #12]	@ D.65714, mod_reg_addr
	ldr	r3, [sp, #8]	@ D.65714, mod_reg_val
	eor	r3, r3, r2	@ D.65714, D.65714, D.65714
	str	r3, [sp, #8]	@ D.65714, mod_reg_val
	ldr	r3, [sp, #8]	@ D.65714, mod_reg_val
	add	r2, r3, #1	@ D.65714, D.65714,
	tst	r3, #8	@ D.65714,
	str	r2, [sp, #8]	@ D.65714, mod_reg_val
	ldreq	r2, [sp, #8]	@ D.65714, mod_reg_val
	ldreq	r3, [sp, #12]	@ D.65714, mod_reg_addr
	muleq	r3, r3, r2	@ D.65714, D.65714, D.65714
	streq	r3, [sp, #12]	@ D.65714, mod_reg_addr
	movw	r3, #819	@ tmp209,
	ldr	r2, [sp, #12]	@ D.65714, mod_reg_addr
	mul	r3, r3, r2	@ D.65714, tmp209, D.65714
	str	r3, [sp, #8]	@ D.65714, mod_reg_val
	ldr	r3, [sp, #8]	@ D.65714, mod_reg_val
	and	r3, r3, #3072	@ D.65714, D.65714,
	str	r3, [sp, #12]	@ D.65714, mod_reg_addr
	ldr	r3, [sp, #8]	@ D.65714, mod_reg_val
	add	r2, r3, #1	@ D.65714, D.65714,
	tst	r3, #8	@ D.65714,
	str	r2, [sp, #8]	@ D.65714, mod_reg_val
	ldreq	r2, [sp, #8]	@ D.65714, mod_reg_val
	ldreq	r3, [sp, #12]	@ D.65714, mod_reg_addr
	muleq	r3, r3, r2	@ D.65714, D.65714, D.65714
	streq	r3, [sp, #12]	@ D.65714, mod_reg_addr
	ldr	r3, [sp, #8]	@ D.65714, mod_reg_val
.L10:
	mov	r3, #9	@ tmp211,
	str	r3, [sp]	@ tmp211, mod_reg_val
	mov	r3, #16	@ tmp212,
	str	r3, [sp, #4]	@ tmp212, mod_reg_addr
	ldr	r3, [sp, #4]	@ D.65714, mod_reg_addr
	str	r3, [sp]	@ D.65714, mod_reg_val
	ldr	r3, [sp, #4]	@ D.65714, mod_reg_addr
	cmp	r3, #3072	@ D.65714,
	ldr	r3, [sp]	@ D.65714, mod_reg_val
	beq	.L1	@,
	cmp	r3, #21504	@ D.65714,
	ldreq	r3, [sp, #4]	@ D.65714, mod_reg_addr
	beq	.L1	@,
	ldr	r3, [sp]	@ D.65714, mod_reg_val
	and	r3, r3, #243	@ D.65714, D.65714,
	str	r3, [sp, #4]	@ D.65714, mod_reg_addr
	ldr	r2, [sp, #4]	@ D.65714, mod_reg_addr
	ldr	r3, [sp]	@ D.65714, mod_reg_val
	eor	r3, r3, r2	@ D.65714, D.65714, D.65714
	str	r3, [sp]	@ D.65714, mod_reg_val
	ldr	r2, [sp]	@ D.65714, mod_reg_val
	ldr	r3, [sp, #4]	@ D.65714, mod_reg_addr
	eor	r3, r3, r2	@ D.65714, D.65714, D.65714
	str	r3, [sp, #4]	@ D.65714, mod_reg_addr
	ldr	r2, [sp, #4]	@ D.65714, mod_reg_addr
	ldr	r3, [sp]	@ D.65714, mod_reg_val
	eor	r3, r3, r2	@ D.65714, D.65714, D.65714
	str	r3, [sp]	@ D.65714, mod_reg_val
	ldr	r3, [sp]	@ D.65714, mod_reg_val
	and	r3, r3, #243	@ D.65714, D.65714,
	str	r3, [sp, #4]	@ D.65714, mod_reg_addr
	ldr	r3, [sp]	@ D.65714, mod_reg_val
	and	r3, r3, #3072	@ D.65714, D.65714,
	str	r3, [sp, #4]	@ D.65714, mod_reg_addr
	ldr	r2, [sp, #4]	@ D.65714, mod_reg_addr
	ldr	r3, [sp]	@ D.65714, mod_reg_val
	eor	r3, r3, r2	@ D.65714, D.65714, D.65714
	str	r3, [sp]	@ D.65714, mod_reg_val
	ldr	r3, [sp]	@ D.65714, mod_reg_val
.L1:
	add	sp, sp, #28	@,,
	@ sp needed	@
	pop	{r4, r5, r6, r7, pc}	@
.L24:
	.align	2
.L23:
	.word	1431655766
	.fnend
	.size	can_asm_write_cmdreg, .-can_asm_write_cmdreg
	.align	2
	.global	can_asm_probe
	.syntax unified
	.arm
	.type	can_asm_probe, %function
can_asm_probe:
	.fnstart
	@ args = 0, pretend = 0, frame = 96
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, lr}	@
	.save {r4, r5, r6, r7, lr}
	.pad #124
	sub	sp, sp, #124	@,,
	ldr	r6, .L34	@ tmp120,
	add	r2, sp, #28	@ tmp125,,
	mov	r5, r0	@ node, node
	str	r2, [sp, #16]	@ tmp125,
	mov	r2, r1	@, num
	ldr	r3, [r6]	@ __stack_chk_guard, __stack_chk_guard
	ldr	r1, .L34+4	@,
	ldr	r0, .L34+8	@,
	str	r3, [sp, #116]	@ __stack_chk_guard, D.65720
	mov	r3, #0	@ tmp124,
	str	r3, [sp, #20]	@ tmp124,
	str	r3, [sp, #12]	@ tmp124,
	str	r3, [sp, #8]	@ tmp124,
	str	r3, [sp, #4]	@ tmp124,
	str	r3, [sp]	@ tmp124,
	bl	__arm_smccc_smc	@
	ldr	r4, [sp, #28]	@ res.a0, res.a0
	cmp	r4, #0	@ res.a0,
	beq	.L26	@,
.L28:
	mvn	r0, #0	@ D.65717,
	b	.L27	@
.L26:
	mov	r2, r4	@, res.a0
	ldr	r1, .L34+12	@,
	mov	r0, r5	@, node
	bl	of_get_property	@
	mov	r7, r0	@ be_addr,
	mov	r0, r5	@, node
	bl	of_irq_find_parent	@
	cmp	r0, #0	@ p
	beq	.L28	@,
	mov	r3, #3	@ tmp132,
	str	r0, [sp, #44]	@ p, oirq.np
	str	r3, [sp, #48]	@ tmp132, oirq.args_count
	mov	r3, #21	@ tmp134,
	str	r3, [sp, #56]	@ tmp134, oirq.args
	add	r1, sp, #44	@ tmp143,,
	mov	r3, #4	@ tmp135,
	mov	r0, r7	@, be_addr
	str	r4, [sp, #52]	@ res.a0, oirq.args
	str	r3, [sp, #60]	@ tmp135, oirq.args
	bl	of_irq_parse_raw	@
	add	r0, sp, #44	@ tmp144,,
	bl	irq_create_of_mapping	@
.L27:
	ldr	r2, [sp, #116]	@ D.65720, D.65720
	ldr	r3, [r6]	@ __stack_chk_guard, __stack_chk_guard
	cmp	r2, r3	@ D.65720, __stack_chk_guard
	beq	.L29	@,
	bl	__stack_chk_fail	@
.L29:
	add	sp, sp, #124	@,,
	@ sp needed	@
	pop	{r4, r5, r6, r7, pc}	@
.L35:
	.align	2
.L34:
	.word	__stack_chk_guard
	.word	-65534
	.word	-1308622748
	.word	.LC0
	.fnend
	.size	can_asm_probe, .-can_asm_probe
	.align	2
	.global	can_asm_fun0
	.syntax unified
	.arm
	.type	can_asm_fun0, %function
can_asm_fun0:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, lr}	@
	.save {r4, lr}
	.pad #48
	sub	sp, sp, #48	@,,
	ldr	r4, .L39	@ tmp112,
	add	r2, sp, #28	@ tmp117,,
	str	r2, [sp, #16]	@ tmp117,
	mov	r2, r0	@, num
	ldr	r3, [r4]	@ __stack_chk_guard, __stack_chk_guard
	ldr	r1, .L39+4	@,
	ldr	r0, .L39+8	@,
	str	r3, [sp, #44]	@ __stack_chk_guard, D.65727
	mov	r3, #0	@ tmp116,
	str	r3, [sp, #20]	@ tmp116,
	str	r3, [sp, #12]	@ tmp116,
	str	r3, [sp, #8]	@ tmp116,
	str	r3, [sp, #4]	@ tmp116,
	str	r3, [sp]	@ tmp116,
	bl	__arm_smccc_smc	@
	ldr	r2, [sp, #44]	@ D.65727, D.65727
	ldr	r3, [r4]	@ __stack_chk_guard, __stack_chk_guard
	cmp	r2, r3	@ D.65727, __stack_chk_guard
	beq	.L37	@,
	bl	__stack_chk_fail	@
.L37:
	add	sp, sp, #48	@,,
	@ sp needed	@
	pop	{r4, pc}	@
.L40:
	.align	2
.L39:
	.word	__stack_chk_guard
	.word	-65529
	.word	-1308622748
	.fnend
	.size	can_asm_fun0, .-can_asm_fun0
	.align	2
	.global	can_asm_fun1
	.syntax unified
	.arm
	.type	can_asm_fun1, %function
can_asm_fun1:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, lr}	@
	.save {r4, lr}
	.pad #48
	sub	sp, sp, #48	@,,
	ldr	r4, .L44	@ tmp112,
	add	r2, sp, #28	@ tmp117,,
	str	r2, [sp, #16]	@ tmp117,
	mov	r2, r0	@, num
	ldr	r3, [r4]	@ __stack_chk_guard, __stack_chk_guard
	ldr	r1, .L44+4	@,
	ldr	r0, .L44+8	@,
	str	r3, [sp, #44]	@ __stack_chk_guard, D.65731
	mov	r3, #0	@ tmp116,
	str	r3, [sp, #20]	@ tmp116,
	str	r3, [sp, #12]	@ tmp116,
	str	r3, [sp, #8]	@ tmp116,
	str	r3, [sp, #4]	@ tmp116,
	str	r3, [sp]	@ tmp116,
	bl	__arm_smccc_smc	@
	ldr	r2, [sp, #44]	@ D.65731, D.65731
	ldr	r3, [r4]	@ __stack_chk_guard, __stack_chk_guard
	cmp	r2, r3	@ D.65731, __stack_chk_guard
	beq	.L42	@,
	bl	__stack_chk_fail	@
.L42:
	add	sp, sp, #48	@,,
	@ sp needed	@
	pop	{r4, pc}	@
.L45:
	.align	2
.L44:
	.word	__stack_chk_guard
	.word	-65533
	.word	-1308622748
	.fnend
	.size	can_asm_fun1, .-can_asm_fun1
	.align	2
	.global	can_asm_fun2
	.syntax unified
	.arm
	.type	can_asm_fun2, %function
can_asm_fun2:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, lr}	@
	.save {r4, lr}
	.pad #48
	sub	sp, sp, #48	@,,
	ldr	r4, .L49	@ tmp112,
	add	r2, sp, #28	@ tmp117,,
	str	r2, [sp, #16]	@ tmp117,
	mov	r2, r0	@, num
	ldr	r3, [r4]	@ __stack_chk_guard, __stack_chk_guard
	ldr	r1, .L49+4	@,
	ldr	r0, .L49+8	@,
	str	r3, [sp, #44]	@ __stack_chk_guard, D.65735
	mov	r3, #0	@ tmp116,
	str	r3, [sp, #20]	@ tmp116,
	str	r3, [sp, #12]	@ tmp116,
	str	r3, [sp, #8]	@ tmp116,
	str	r3, [sp, #4]	@ tmp116,
	str	r3, [sp]	@ tmp116,
	bl	__arm_smccc_smc	@
	ldr	r2, [sp, #44]	@ D.65735, D.65735
	ldr	r3, [r4]	@ __stack_chk_guard, __stack_chk_guard
	cmp	r2, r3	@ D.65735, __stack_chk_guard
	beq	.L47	@,
	bl	__stack_chk_fail	@
.L47:
	add	sp, sp, #48	@,,
	@ sp needed	@
	pop	{r4, pc}	@
.L50:
	.align	2
.L49:
	.word	__stack_chk_guard
	.word	-65532
	.word	-1308622748
	.fnend
	.size	can_asm_fun2, .-can_asm_fun2
	.align	2
	.global	can_asm_fun3
	.syntax unified
	.arm
	.type	can_asm_fun3, %function
can_asm_fun3:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, lr}	@
	.save {r4, lr}
	.pad #48
	sub	sp, sp, #48	@,,
	ldr	r4, .L54	@ tmp111,
	add	r2, sp, #28	@ tmp116,,
	str	r2, [sp, #16]	@ tmp116,
	ldr	r3, [r4]	@ __stack_chk_guard, __stack_chk_guard
	ldr	r1, .L54+4	@,
	ldr	r0, .L54+8	@,
	str	r3, [sp, #44]	@ __stack_chk_guard, D.65738
	mov	r3, #0	@ tmp115,
	mov	r2, r3	@,
	str	r3, [sp, #20]	@ tmp115,
	str	r3, [sp, #12]	@ tmp115,
	str	r3, [sp, #8]	@ tmp115,
	str	r3, [sp, #4]	@ tmp115,
	str	r3, [sp]	@ tmp115,
	bl	__arm_smccc_smc	@
	ldr	r2, [sp, #44]	@ D.65738, D.65738
	ldr	r3, [r4]	@ __stack_chk_guard, __stack_chk_guard
	cmp	r2, r3	@ D.65738, __stack_chk_guard
	beq	.L52	@,
	bl	__stack_chk_fail	@
.L52:
	add	sp, sp, #48	@,,
	@ sp needed	@
	pop	{r4, pc}	@
.L55:
	.align	2
.L54:
	.word	__stack_chk_guard
	.word	-65531
	.word	-1308622748
	.fnend
	.size	can_asm_fun3, .-can_asm_fun3
	.align	2
	.global	can_asm_fun4
	.syntax unified
	.arm
	.type	can_asm_fun4, %function
can_asm_fun4:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, lr}	@
	.save {r4, lr}
	.pad #48
	sub	sp, sp, #48	@,,
	ldr	r4, .L59	@ tmp111,
	add	r2, sp, #28	@ tmp116,,
	str	r2, [sp, #16]	@ tmp116,
	ldr	r3, [r4]	@ __stack_chk_guard, __stack_chk_guard
	ldr	r1, .L59+4	@,
	ldr	r0, .L59+8	@,
	str	r3, [sp, #44]	@ __stack_chk_guard, D.65741
	mov	r3, #0	@ tmp115,
	mov	r2, r3	@,
	str	r3, [sp, #20]	@ tmp115,
	str	r3, [sp, #12]	@ tmp115,
	str	r3, [sp, #8]	@ tmp115,
	str	r3, [sp, #4]	@ tmp115,
	str	r3, [sp]	@ tmp115,
	bl	__arm_smccc_smc	@
	ldr	r2, [sp, #44]	@ D.65741, D.65741
	ldr	r3, [r4]	@ __stack_chk_guard, __stack_chk_guard
	cmp	r2, r3	@ D.65741, __stack_chk_guard
	beq	.L57	@,
	bl	__stack_chk_fail	@
.L57:
	add	sp, sp, #48	@,,
	@ sp needed	@
	pop	{r4, pc}	@
.L60:
	.align	2
.L59:
	.word	__stack_chk_guard
	.word	-65530
	.word	-1308622748
	.fnend
	.size	can_asm_fun4, .-can_asm_fun4
	.align	2
	.global	can_asm_fun5
	.syntax unified
	.arm
	.type	can_asm_fun5, %function
can_asm_fun5:
	.fnstart
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, lr}	@
	.save {r4, lr}
	.pad #48
	sub	sp, sp, #48	@,,
	ldr	r4, .L64	@ tmp116,
	add	r2, sp, #28	@ tmp121,,
	str	r2, [sp, #16]	@ tmp121,
	mov	r2, r0	@, num
	ldr	r3, [r4]	@ __stack_chk_guard, __stack_chk_guard
	ldr	r1, .L64+4	@,
	ldr	r0, .L64+8	@,
	str	r3, [sp, #44]	@ __stack_chk_guard, D.65746
	mov	r3, #0	@ tmp120,
	str	r3, [sp, #20]	@ tmp120,
	str	r3, [sp, #12]	@ tmp120,
	str	r3, [sp, #8]	@ tmp120,
	str	r3, [sp, #4]	@ tmp120,
	str	r3, [sp]	@ tmp120,
	bl	__arm_smccc_smc	@
	ldr	r1, [sp, #36]	@, res.a2
	ldr	r0, [sp, #32]	@, res.a1
	bl	ioremap	@
	ldr	r2, [sp, #44]	@ D.65746, D.65746
	ldr	r3, [r4]	@ __stack_chk_guard, __stack_chk_guard
	cmp	r2, r3	@ D.65746, __stack_chk_guard
	beq	.L62	@,
	bl	__stack_chk_fail	@
.L62:
	add	sp, sp, #48	@,,
	@ sp needed	@
	pop	{r4, pc}	@
.L65:
	.align	2
.L64:
	.word	__stack_chk_guard
	.word	-65528
	.word	-1308622748
	.fnend
	.size	can_asm_fun5, .-can_asm_fun5
	.align	2
	.global	can_asm_start
	.syntax unified
	.arm
	.type	can_asm_start, %function
can_asm_start:
	.fnstart
	@ args = 0, pretend = 0, frame = 80
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, lr}	@
	.save {r4, r5, r6, r7, lr}
	mov	r3, r1	@ ctrlmode, ctrlmode
	ldr	r4, .L106	@ tmp270,
	.pad #108
	sub	sp, sp, #108	@,,
	mov	r5, r0	@ mod_reg_addr, mod_reg_addr
	ldr	r1, [r4]	@ __stack_chk_guard, __stack_chk_guard
	str	r1, [sp, #100]	@ __stack_chk_guard, D.65774
	mov	r1, #3	@ tmp272,
	str	r1, [sp, #28]	@ tmp272, mod_reg_val
	mov	r1, #15	@ tmp273,
	str	r1, [sp, #32]	@ tmp273, mod_reg_addr
	ldr	r1, [sp, #28]	@ D.65770, mod_reg_val
	and	r1, r1, #3072	@ D.65770, D.65770,
	str	r1, [sp, #32]	@ D.65770, mod_reg_addr
	ldr	r1, [sp, #28]	@ D.65770, mod_reg_val
	cmp	r1, #21504	@ D.65770,
	ldreq	r1, [sp, #32]	@ D.65770, mod_reg_addr
	beq	.L68	@,
	ldr	r0, [sp, #28]	@ D.65770, mod_reg_val
	ldr	r1, [sp, #32]	@ D.65770, mod_reg_addr
	cmp	r0, r1	@ D.65770, D.65770
	bge	.L69	@,
	ldr	r0, [sp, #28]	@ D.65770, mod_reg_val
	ldr	r6, .L106+4	@ tmp276,
	ldr	r1, [sp, #32]	@ D.65770, mod_reg_addr
	smull	r6, r7, r0, r6	@ tmp275, D.65770, tmp276
	sub	r0, r7, r0, asr #31	@ D.65770,, D.65770,
	add	r0, r0, r1	@ D.65770, D.65770, D.65770
	str	r0, [sp, #32]	@ D.65770, mod_reg_addr
.L69:
	ldr	r1, [sp, #28]	@ D.65770, mod_reg_val
	add	r0, r1, #1	@ D.65770, D.65770,
	tst	r1, #8	@ D.65770,
	str	r0, [sp, #28]	@ D.65770, mod_reg_val
	ldreq	r1, [sp, #28]	@ D.65770, mod_reg_val
	ldreq	r0, [sp, #32]	@ D.65770, mod_reg_addr
	muleq	r0, r0, r1	@ D.65770, D.65770, D.65770
	streq	r0, [sp, #32]	@ D.65770, mod_reg_addr
	movw	r0, #819	@ tmp279,
	ldr	r1, [sp, #32]	@ D.65770, mod_reg_addr
	mul	r0, r0, r1	@ D.65770, tmp279, D.65770
	str	r0, [sp, #28]	@ D.65770, mod_reg_val
	ldr	r1, [sp, #28]	@ D.65770, mod_reg_val
.L68:
	mov	r0, #0	@ tmp282,
	add	r1, sp, #84	@ tmp283,,
	str	r0, [sp, #20]	@ tmp282,
	str	r1, [sp, #16]	@ tmp283,
	str	r0, [sp, #12]	@ tmp282,
	str	r0, [sp, #8]	@ tmp282,
	str	r0, [sp, #4]	@ tmp282,
	str	r0, [sp]	@ tmp282,
	ldr	r1, .L106+8	@,
	ldr	r0, .L106+12	@,
	bl	__arm_smccc_smc	@
	ldr	r3, [sp, #84]	@ res.a0, res.a0
	cmp	r3, #0	@ res.a0,
	mov	r3, #15	@ tmp289,
	str	r3, [sp, #44]	@ tmp289, mod_reg_val
	mov	r3, #6	@ tmp290,
	str	r3, [sp, #48]	@ tmp290, mod_reg_addr
	moveq	r6, #24576	@ mod_reg_val,
	ldr	r3, [sp, #44]	@ D.65770, mod_reg_val
	movne	r6, #384	@ mod_reg_val,
	cmp	r3, #21504	@ D.65770,
	beq	.L102	@,
	ldr	r2, [sp, #44]	@ D.65770, mod_reg_val
	ldr	r3, [sp, #48]	@ D.65770, mod_reg_addr
	cmp	r2, r3	@ D.65770, D.65770
	blt	.L102	@,
	ldr	r2, [sp, #44]	@ D.65770, mod_reg_val
	ldr	r3, [sp, #48]	@ D.65770, mod_reg_addr
	eor	r3, r3, r2	@ D.65770, D.65770, D.65770
	str	r3, [sp, #48]	@ D.65770, mod_reg_addr
	ldr	r3, [sp, #44]	@ D.65770, mod_reg_val
	add	r2, r3, #1	@ D.65770, D.65770,
	tst	r3, #8	@ D.65770,
	str	r2, [sp, #44]	@ D.65770, mod_reg_val
	ldreq	r2, [sp, #44]	@ D.65770, mod_reg_val
	ldreq	r3, [sp, #48]	@ D.65770, mod_reg_addr
	muleq	r3, r3, r2	@ D.65770, D.65770, D.65770
	streq	r3, [sp, #48]	@ D.65770, mod_reg_addr
	movw	r3, #770	@ tmp292,
	ldr	r2, [sp, #48]	@ D.65770, mod_reg_addr
	cmp	r2, r3	@ D.65770, tmp292
	ldrne	r3, [sp, #44]	@ D.65770, mod_reg_val
	ldrne	r2, [sp, #48]	@ D.65770, mod_reg_addr
	addne	r3, r3, r2	@ D.65770, D.65770, D.65770
	strne	r3, [sp, #48]	@ D.65770, mod_reg_addr
	ldr	r3, [sp, #44]	@ D.65770, mod_reg_val
	tst	r3, #8	@ D.65770,
	add	r2, r3, #1	@ D.65770, D.65770,
	str	r2, [sp, #44]	@ D.65770, mod_reg_val
	ldreq	r2, [sp, #44]	@ D.65770, mod_reg_val
	ldreq	r3, [sp, #48]	@ D.65770, mod_reg_addr
	muleq	r3, r3, r2	@ D.65770, D.65770, D.65770
	streq	r3, [sp, #48]	@ D.65770, mod_reg_addr
	ldr	r3, [sp, #44]	@ D.65770, mod_reg_val
	cmp	r3, #21504	@ D.65770,
	bne	.L78	@,
.L102:
	ldr	r3, [sp, #48]	@ D.65770, mod_reg_addr
	b	.L73	@
.L78:
	ldr	r3, [sp, #44]	@ D.65770, mod_reg_val
	add	r3, r3, #1	@ D.65770, D.65770,
	str	r3, [sp, #44]	@ D.65770, mod_reg_val
	ldr	r2, [sp, #44]	@ D.65770, mod_reg_val
	ldr	r3, [sp, #48]	@ D.65770, mod_reg_addr
	cmp	r2, r3	@ D.65770, D.65770
	bge	.L79	@,
	ldr	r3, [sp, #44]	@ D.65770, mod_reg_val
	ldr	r0, .L106+4	@ tmp296,
	ldr	r2, [sp, #48]	@ D.65770, mod_reg_addr
	smull	r0, r1, r3, r0	@ tmp295, D.65770, tmp296
	sub	r3, r1, r3, asr #31	@ D.65770,, D.65770,
	add	r3, r3, r2	@ D.65770, D.65770, D.65770
	str	r3, [sp, #48]	@ D.65770, mod_reg_addr
.L79:
	ldr	r3, [sp, #44]	@ D.65770, mod_reg_val
	add	r2, r3, #1	@ D.65770, D.65770,
	tst	r3, #8	@ D.65770,
	str	r2, [sp, #44]	@ D.65770, mod_reg_val
	ldreq	r2, [sp, #44]	@ D.65770, mod_reg_val
	ldreq	r3, [sp, #48]	@ D.65770, mod_reg_addr
	muleq	r3, r3, r2	@ D.65770, D.65770, D.65770
	streq	r3, [sp, #48]	@ D.65770, mod_reg_addr
	ldr	r3, [sp, #44]	@ D.65770, mod_reg_val
	add	r2, r3, #1	@ D.65770, D.65770,
	tst	r3, #8	@ D.65770,
	str	r2, [sp, #44]	@ D.65770, mod_reg_val
	ldreq	r2, [sp, #44]	@ D.65770, mod_reg_val
	ldreq	r3, [sp, #48]	@ D.65770, mod_reg_addr
	muleq	r3, r3, r2	@ D.65770, D.65770, D.65770
	streq	r3, [sp, #48]	@ D.65770, mod_reg_addr
	ldr	r3, [sp, #48]	@ D.65770, mod_reg_addr
	str	r3, [sp, #44]	@ D.65770, mod_reg_val
	ldr	r3, [sp, #44]	@ D.65770, mod_reg_val
	and	r3, r3, #3072	@ D.65770, D.65770,
	str	r3, [sp, #48]	@ D.65770, mod_reg_addr
	ldr	r2, [sp, #48]	@ D.65770, mod_reg_addr
	ldr	r3, [sp, #44]	@ D.65770, mod_reg_val
	eor	r3, r3, r2	@ D.65770, D.65770, D.65770
	str	r3, [sp, #44]	@ D.65770, mod_reg_val
	ldr	r3, [sp, #44]	@ D.65770, mod_reg_val
	and	r3, r3, #243	@ D.65770, D.65770,
	str	r3, [sp, #48]	@ D.65770, mod_reg_addr
	ldr	r3, [sp, #44]	@ D.65770, mod_reg_val
	add	r3, r3, #1	@ D.65770, D.65770,
	str	r3, [sp, #44]	@ D.65770, mod_reg_val
	ldr	r3, [sp, #44]	@ D.65770, mod_reg_val
.L73:
	mov	r3, #3072	@ tmp300,
	str	r3, [sp, #36]	@ tmp300, mod_reg_val
	mov	r3, #21504	@ tmp301,
	str	r3, [sp, #40]	@ tmp301, mod_reg_addr
	ldr	r3, [sp, #40]	@ D.65770, mod_reg_addr
	add	r3, r3, #100	@ D.65770, D.65770,
	str	r3, [sp, #40]	@ D.65770, mod_reg_addr
	ldr	r3, [sp, #40]	@ D.65770, mod_reg_addr
	sub	r3, r3, #100	@ D.65770, D.65770,
	str	r3, [sp, #40]	@ D.65770, mod_reg_addr
	ldr	r3, [sp, #36]	@ D.65770, mod_reg_val
	ldr	r2, [sp, #40]	@ D.65770, mod_reg_addr
	add	r3, r3, r2	@ D.65770, D.65770, D.65770
	cmp	r6, r3	@ mod_reg_val, D.65770
	bcc	.L103	@,
	movw	r3, #1638	@ tmp304,
	str	r3, [sp, #52]	@ tmp304, mod_reg_val
	movw	r3, #22938	@ tmp305,
	str	r3, [sp, #56]	@ tmp305, mod_reg_addr
	ldr	r3, [sp, #52]	@ D.65770, mod_reg_val
	ldr	r2, [sp, #56]	@ D.65770, mod_reg_addr
	add	r3, r3, r2	@ D.65770, D.65770, D.65770
	cmp	r6, r3	@ mod_reg_val, D.65770
	bls	.L83	@,
.L103:
	.syntax divided
@ 548 "drivers/net/can/sunxi_can_asm.c" 1
	dsb st
@ 0 "" 2
	.arm
	.syntax unified
	bl	arm_heavy_mb	@
	mov	r3, #48	@ tmp307,
	.syntax divided
@ 95 "./arch/arm/include/asm/io.h" 1
	str r3, [r5, #16]	@ tmp307,
@ 0 "" 2
	.arm
	.syntax unified
.L83:
	mov	r3, #7	@ tmp308,
	str	r3, [sp, #68]	@ tmp308, mod_reg_val
	mov	r3, #1	@ tmp309,
	str	r3, [sp, #72]	@ tmp309, mod_reg_addr
	ldr	r2, [sp, #68]	@ D.65770, mod_reg_val
	ldr	r3, [sp, #72]	@ D.65770, mod_reg_addr
	cmp	r2, r3	@ D.65770, D.65770
	bge	.L84	@,
	ldr	r3, [sp, #68]	@ D.65770, mod_reg_val
	ldr	r0, .L106+4	@ tmp312,
	ldr	r2, [sp, #72]	@ D.65770, mod_reg_addr
	smull	r0, r1, r3, r0	@ tmp311, D.65770, tmp312
	sub	r3, r1, r3, asr #31	@ D.65770,, D.65770,
	add	r3, r3, r2	@ D.65770, D.65770, D.65770
	str	r3, [sp, #72]	@ D.65770, mod_reg_addr
.L84:
	ldr	r2, [sp, #72]	@ D.65770, mod_reg_addr
	movw	r3, #770	@ tmp314,
	cmp	r2, r3	@ D.65770, tmp314
	ldrne	r3, [sp, #68]	@ D.65770, mod_reg_val
	ldrne	r2, [sp, #72]	@ D.65770, mod_reg_addr
	addne	r3, r3, r2	@ D.65770, D.65770, D.65770
	strne	r3, [sp, #72]	@ D.65770, mod_reg_addr
	ldr	r3, [sp, #72]	@ D.65770, mod_reg_addr
	cmp	r3, #3072	@ D.65770,
	beq	.L104	@,
	ldr	r2, [sp, #68]	@ D.65770, mod_reg_val
	ldr	r3, [sp, #72]	@ D.65770, mod_reg_addr
	eor	r3, r3, r2	@ D.65770, D.65770, D.65770
	str	r3, [sp, #72]	@ D.65770, mod_reg_addr
	ldr	r3, [sp, #68]	@ D.65770, mod_reg_val
	cmp	r3, #3072	@ D.65770,
	ldr	r3, [sp, #72]	@ D.65770, mod_reg_addr
	beq	.L87	@,
	cmp	r3, #3072	@ D.65770,
	beq	.L104	@,
	ldr	r3, [sp, #72]	@ D.65770, mod_reg_addr
	str	r3, [sp, #68]	@ D.65770, mod_reg_val
	ldr	r3, [sp, #68]	@ D.65770, mod_reg_val
	add	r3, r3, #1	@ D.65770, D.65770,
	str	r3, [sp, #68]	@ D.65770, mod_reg_val
	ldr	r3, [sp, #72]	@ D.65770, mod_reg_addr
	add	r3, r3, #51	@ D.65770, D.65770,
	str	r3, [sp, #72]	@ D.65770, mod_reg_addr
	ldr	r3, [sp, #72]	@ D.65770, mod_reg_addr
	cmp	r3, #21504	@ D.65770,
	beq	.L105	@,
	ldr	r3, [sp, #68]	@ D.65770, mod_reg_val
	and	r3, r3, #3072	@ D.65770, D.65770,
	str	r3, [sp, #72]	@ D.65770, mod_reg_addr
	ldr	r2, [sp, #68]	@ D.65770, mod_reg_val
	ldr	r3, [sp, #72]	@ D.65770, mod_reg_addr
	cmp	r2, r3	@ D.65770, D.65770
	blt	.L105	@,
	ldr	r3, [sp, #68]	@ D.65770, mod_reg_val
	cmp	r3, #21504	@ D.65770,
	beq	.L105	@,
	ldr	r3, [sp, #68]	@ D.65770, mod_reg_val
	add	r3, r3, #1	@ D.65770, D.65770,
	str	r3, [sp, #68]	@ D.65770, mod_reg_val
	ldr	r3, [sp, #72]	@ D.65770, mod_reg_addr
	add	r3, r3, #51	@ D.65770, D.65770,
	str	r3, [sp, #72]	@ D.65770, mod_reg_addr
	ldr	r3, [sp, #72]	@ D.65770, mod_reg_addr
	cmp	r3, #21504	@ D.65770,
	beq	.L105	@,
	ldr	r2, [sp, #68]	@ D.65770, mod_reg_val
	ldr	r3, [sp, #72]	@ D.65770, mod_reg_addr
	cmp	r2, r3	@ D.65770, D.65770
	bge	.L94	@,
.L105:
	ldr	r3, [sp, #72]	@ D.65770, mod_reg_addr
	b	.L87	@
.L94:
	ldr	r2, [sp, #72]	@ D.65770, mod_reg_addr
	ldr	r3, [sp, #68]	@ D.65770, mod_reg_val
	eor	r3, r3, r2	@ D.65770, D.65770, D.65770
	str	r3, [sp, #68]	@ D.65770, mod_reg_val
.L104:
	ldr	r3, [sp, #68]	@ D.65770, mod_reg_val
.L87:
	movw	r3, #2457	@ tmp315,
	str	r3, [sp, #60]	@ tmp315, mod_reg_val
	movw	r3, #22119	@ tmp316,
	str	r3, [sp, #64]	@ tmp316, mod_reg_addr
	ldr	r3, [sp, #60]	@ D.65770, mod_reg_val
	ldr	r2, [sp, #64]	@ D.65770, mod_reg_addr
	add	r3, r3, r2	@ D.65770, D.65770, D.65770
	str	r3, [sp, #64]	@ D.65770, mod_reg_addr
	ldr	r2, [sp, #60]	@ D.65770, mod_reg_val
	ldr	r3, [sp, #64]	@ D.65770, mod_reg_addr
	sub	r3, r3, r2	@ D.65770, D.65770, D.65770
	str	r3, [sp, #64]	@ D.65770, mod_reg_addr
	ldr	r3, [sp, #60]	@ D.65770, mod_reg_val
	ldr	r2, [sp, #64]	@ D.65770, mod_reg_addr
	add	r3, r3, r2	@ D.65770, D.65770, D.65770
	cmp	r6, r3	@ mod_reg_val, D.65770
	beq	.L95	@,
	.syntax divided
@ 550 "drivers/net/can/sunxi_can_asm.c" 1
	dsb st
@ 0 "" 2
	.arm
	.syntax unified
	bl	arm_heavy_mb	@
	mov	r3, #4	@ tmp318,
	.syntax divided
@ 95 "./arch/arm/include/asm/io.h" 1
	str r3, [r5]	@ tmp318, MEM[(volatile u32 *)mod_reg_addr_18(D)]
@ 0 "" 2
@ 550 "drivers/net/can/sunxi_can_asm.c" 1
	dsb st
@ 0 "" 2
	.arm
	.syntax unified
	bl	arm_heavy_mb	@
	mov	r3, #48	@ tmp319,
	.syntax divided
@ 95 "./arch/arm/include/asm/io.h" 1
	str r3, [r5, #16]	@ tmp319, MEM[(volatile u32 *)_34]
@ 0 "" 2
	.arm
	.syntax unified
.L95:
	mov	r3, #3	@ tmp320,
	str	r3, [sp, #76]	@ tmp320, mod_reg_val
	mov	r3, #10	@ tmp321,
	str	r3, [sp, #80]	@ tmp321, mod_reg_addr
	ldr	r3, [sp, #76]	@ D.65770, mod_reg_val
	cmp	r3, #21504	@ D.65770,
	ldrne	r3, [sp, #76]	@ D.65770, mod_reg_val
	ldreq	r3, [sp, #80]	@ D.65770, mod_reg_addr
	andne	r3, r3, #3072	@ D.65770, D.65770,
	strne	r3, [sp, #80]	@ D.65770, mod_reg_addr
	ldrne	r3, [sp, #80]	@ D.65770, mod_reg_addr
	strne	r3, [sp, #76]	@ D.65770, mod_reg_val
	ldrne	r3, [sp, #80]	@ D.65770, mod_reg_addr
	strne	r3, [sp, #76]	@ D.65770, mod_reg_val
	ldrne	r3, [sp, #76]	@ D.65770, mod_reg_val
	ldr	r2, [sp, #100]	@ D.65774, D.65774
	ldr	r3, [r4]	@ __stack_chk_guard, __stack_chk_guard
	cmp	r2, r3	@ D.65774, __stack_chk_guard
	beq	.L98	@,
	bl	__stack_chk_fail	@
.L98:
	add	sp, sp, #108	@,,
	@ sp needed	@
	pop	{r4, r5, r6, r7, pc}	@
.L107:
	.align	2
.L106:
	.word	__stack_chk_guard
	.word	1431655766
	.word	-65535
	.word	-1308622748
	.fnend
	.size	can_asm_start, .-can_asm_start
	.align	2
	.global	can_asm_set_bittiming
	.syntax unified
	.arm
	.type	can_asm_set_bittiming, %function
can_asm_set_bittiming:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, lr}	@
	.save {r4, r5, lr}
	mov	r3, #16	@ tmp158,
	.pad #20
	sub	sp, sp, #20	@,,
	mov	r4, r2	@ cfg, cfg
	mov	r5, r0	@ mod_reg_addr, mod_reg_addr
	str	r3, [sp]	@ tmp158, mod_reg_val
	mov	r3, #15	@ tmp159,
	str	r3, [sp, #4]	@ tmp159, mod_reg_addr
	ldr	r3, [sp]	@ D.65783, mod_reg_val
	and	r3, r3, #243	@ D.65783, D.65783,
	str	r3, [sp, #4]	@ D.65783, mod_reg_addr
	ldr	r3, [sp]	@ D.65783, mod_reg_val
	and	r3, r3, #3072	@ D.65783, D.65783,
	str	r3, [sp, #4]	@ D.65783, mod_reg_addr
	ldr	r3, [sp]	@ D.65783, mod_reg_val
	and	r3, r3, #3072	@ D.65783, D.65783,
	str	r3, [sp, #4]	@ D.65783, mod_reg_addr
	ldr	r3, [sp]	@ D.65783, mod_reg_val
	and	r3, r3, #243	@ D.65783, D.65783,
	str	r3, [sp, #4]	@ D.65783, mod_reg_addr
	ldr	r2, [sp, #4]	@ D.65783, mod_reg_addr
	ldr	r3, [sp]	@ D.65783, mod_reg_val
	eor	r3, r3, r2	@ D.65783, D.65783, D.65783
	str	r3, [sp]	@ D.65783, mod_reg_val
	movw	r3, #819	@ tmp160,
	ldr	r2, [sp, #4]	@ D.65783, mod_reg_addr
	mul	r3, r3, r2	@ D.65783, tmp160, D.65783
	str	r3, [sp]	@ D.65783, mod_reg_val
	ldr	r3, [sp, #4]	@ D.65783, mod_reg_addr
	cmp	r3, #3072	@ D.65783,
	ldr	r3, [sp]	@ D.65783, mod_reg_val
	beq	.L110	@,
	cmp	r3, #21504	@ D.65783,
	ldrne	r2, [sp, #4]	@ D.65783, mod_reg_addr
	ldrne	r3, [sp]	@ D.65783, mod_reg_val
	ldreq	r3, [sp, #4]	@ D.65783, mod_reg_addr
	eorne	r3, r3, r2	@ D.65783, D.65783, D.65783
	strne	r3, [sp]	@ D.65783, mod_reg_val
	ldrne	r3, [sp]	@ D.65783, mod_reg_val
.L110:
	tst	r1, #4	@ mod_reg_val,
	ldrne	r3, [r4]	@ *cfg_6(D), *cfg_6(D)
	orrne	r3, r3, #8388608	@ D.65784, *cfg_6(D),
	strne	r3, [r4]	@ D.65784, *cfg_6(D)
	mov	r3, #11	@ tmp164,
	str	r3, [sp, #8]	@ tmp164, mod_reg_val
	mov	r3, #4	@ tmp165,
	str	r3, [sp, #12]	@ tmp165, mod_reg_addr
	ldr	r3, [sp, #8]	@ D.65783, mod_reg_val
	add	r3, r3, #1	@ D.65783, D.65783,
	str	r3, [sp, #8]	@ D.65783, mod_reg_val
	ldr	r3, [sp, #8]	@ D.65783, mod_reg_val
	tst	r3, #8	@ D.65783,
	add	r2, r3, #1	@ D.65783, D.65783,
	str	r2, [sp, #8]	@ D.65783, mod_reg_val
	ldreq	r2, [sp, #8]	@ D.65783, mod_reg_val
	ldreq	r3, [sp, #12]	@ D.65783, mod_reg_addr
	muleq	r3, r3, r2	@ D.65783, D.65783, D.65783
	streq	r3, [sp, #12]	@ D.65783, mod_reg_addr
	ldr	r3, [sp, #8]	@ D.65783, mod_reg_val
	add	r3, r3, #1	@ D.65783, D.65783,
	str	r3, [sp, #8]	@ D.65783, mod_reg_val
	ldr	r3, [sp, #12]	@ D.65783, mod_reg_addr
	add	r3, r3, #51	@ D.65783, D.65783,
	str	r3, [sp, #12]	@ D.65783, mod_reg_addr
	ldr	r3, [sp, #12]	@ D.65783, mod_reg_addr
	cmp	r3, #21504	@ D.65783,
	beq	.L121	@,
	ldr	r2, [sp, #8]	@ D.65783, mod_reg_val
	ldr	r3, [sp, #12]	@ D.65783, mod_reg_addr
	cmp	r2, r3	@ D.65783, D.65783
	ldrge	r3, [sp, #8]	@ D.65783, mod_reg_val
	bge	.L115	@,
.L121:
	ldr	r3, [sp, #12]	@ D.65783, mod_reg_addr
.L115:
	.syntax divided
@ 564 "drivers/net/can/sunxi_can_asm.c" 1
	dsb st
@ 0 "" 2
	.arm
	.syntax unified
	bl	arm_heavy_mb	@
	ldr	r3, [r4]	@ D.65784, *cfg_6(D)
	.syntax divided
@ 95 "./arch/arm/include/asm/io.h" 1
	str r3, [r5, #20]	@ D.65784, MEM[(volatile u32 *)_15]
@ 0 "" 2
	.arm
	.syntax unified
	add	sp, sp, #20	@,,
	@ sp needed	@
	pop	{r4, r5, pc}	@
	.fnend
	.size	can_asm_set_bittiming, .-can_asm_set_bittiming
	.align	2
	.global	can_asm_clean_transfer_err
	.syntax unified
	.arm
	.type	can_asm_clean_transfer_err, %function
can_asm_clean_transfer_err:
	.fnstart
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	@ link register save eliminated.
	.pad #16
	sub	sp, sp, #16	@,,
	mov	r3, #10	@ tmp192,
	str	r3, [sp, #8]	@ tmp192, mod_reg_val
	mov	r3, #14	@ tmp193,
	str	r3, [sp, #12]	@ tmp193, mod_reg_addr
	ldr	r3, [sp, #12]	@ D.65795, mod_reg_addr
	cmp	r3, #3072	@ D.65795,
	ldr	r3, [sp, #8]	@ D.65795, mod_reg_val
	beq	.L124	@,
	cmp	r3, #21504	@ D.65795,
	ldreq	r3, [sp, #12]	@ D.65795, mod_reg_addr
	beq	.L124	@,
	ldr	ip, [sp, #8]	@ D.65795, mod_reg_val
	ldr	r3, [sp, #12]	@ D.65795, mod_reg_addr
	eor	r3, r3, ip	@ D.65795, D.65795, D.65795
	str	r3, [sp, #12]	@ D.65795, mod_reg_addr
	movw	r3, #819	@ tmp194,
	ldr	ip, [sp, #12]	@ D.65795, mod_reg_addr
	mul	r3, r3, ip	@ D.65795, tmp194, D.65795
	str	r3, [sp, #8]	@ D.65795, mod_reg_val
	ldr	r3, [sp, #8]	@ D.65795, mod_reg_val
	add	ip, r3, #1	@ D.65795, D.65795,
	tst	r3, #8	@ D.65795,
	str	ip, [sp, #8]	@ D.65795, mod_reg_val
	ldreq	ip, [sp, #8]	@ D.65795, mod_reg_val
	ldreq	r3, [sp, #12]	@ D.65795, mod_reg_addr
	muleq	r3, r3, ip	@ D.65795, D.65795, D.65795
	streq	r3, [sp, #12]	@ D.65795, mod_reg_addr
	movw	r3, #770	@ tmp196,
	ldr	ip, [sp, #12]	@ D.65795, mod_reg_addr
	cmp	ip, r3	@ D.65795, tmp196
	ldrne	r3, [sp, #8]	@ D.65795, mod_reg_val
	ldrne	ip, [sp, #12]	@ D.65795, mod_reg_addr
	addne	r3, r3, ip	@ D.65795, D.65795, D.65795
	strne	r3, [sp, #12]	@ D.65795, mod_reg_addr
	ldr	r3, [sp, #12]	@ D.65795, mod_reg_addr
	str	r3, [sp, #8]	@ D.65795, mod_reg_val
	ldr	r3, [sp, #8]	@ D.65795, mod_reg_val
	and	r3, r3, #3072	@ D.65795, D.65795,
	str	r3, [sp, #12]	@ D.65795, mod_reg_addr
	ldr	ip, [sp, #12]	@ D.65795, mod_reg_addr
	ldr	r3, [sp, #8]	@ D.65795, mod_reg_val
	eor	r3, r3, ip	@ D.65795, D.65795, D.65795
	str	r3, [sp, #8]	@ D.65795, mod_reg_val
	ldr	r3, [sp, #8]	@ D.65795, mod_reg_val
	add	r3, r3, #1	@ D.65795, D.65795,
	str	r3, [sp, #8]	@ D.65795, mod_reg_val
	ldr	r3, [sp, #12]	@ D.65795, mod_reg_addr
	add	r3, r3, #51	@ D.65795, D.65795,
	str	r3, [sp, #12]	@ D.65795, mod_reg_addr
	ldr	r3, [sp, #12]	@ D.65795, mod_reg_addr
	cmp	r3, #21504	@ D.65795,
	ldr	r3, [sp, #12]	@ D.65795, mod_reg_addr
	strne	r3, [sp, #8]	@ D.65795, mod_reg_val
	ldrne	r3, [sp, #8]	@ D.65795, mod_reg_val
.L124:
	.syntax divided
@ 113 "./arch/arm/include/asm/io.h" 1
	ldr r3, [r0, #28]	@ errors, MEM[(volatile u32 *)_4]
@ 0 "" 2
@ 575 "drivers/net/can/sunxi_can_asm.c" 1
	dsb
@ 0 "" 2
	.arm
	.syntax unified
	mov	r0, #1	@ tmp198,
	str	r0, [sp]	@ tmp198, mod_reg_val
	mov	r0, #10	@ tmp199,
	str	r0, [sp, #4]	@ tmp199, mod_reg_addr
	ldr	r0, [sp]	@ D.65795, mod_reg_val
	cmp	r0, #3072	@ D.65795,
	beq	.L139	@,
	ldr	ip, [sp, #4]	@ D.65795, mod_reg_addr
	ldr	r0, [sp]	@ D.65795, mod_reg_val
	eor	r0, r0, ip	@ D.65795, D.65795, D.65795
	str	r0, [sp]	@ D.65795, mod_reg_val
	ldr	r0, [sp]	@ D.65795, mod_reg_val
	add	ip, r0, #1	@ D.65795, D.65795,
	tst	r0, #8	@ D.65795,
	str	ip, [sp]	@ D.65795, mod_reg_val
	ldreq	ip, [sp]	@ D.65795, mod_reg_val
	ldreq	r0, [sp, #4]	@ D.65795, mod_reg_addr
	muleq	r0, r0, ip	@ D.65795, D.65795, D.65795
	streq	r0, [sp, #4]	@ D.65795, mod_reg_addr
	movw	r0, #819	@ tmp201,
	ldr	ip, [sp, #4]	@ D.65795, mod_reg_addr
	mul	r0, r0, ip	@ D.65795, tmp201, D.65795
	str	r0, [sp]	@ D.65795, mod_reg_val
	ldr	r0, [sp, #4]	@ D.65795, mod_reg_addr
	str	r0, [sp]	@ D.65795, mod_reg_val
	ldr	r0, [sp]	@ D.65795, mod_reg_val
	add	ip, r0, #1	@ D.65795, D.65795,
	tst	r0, #8	@ D.65795,
	str	ip, [sp]	@ D.65795, mod_reg_val
	ldreq	ip, [sp]	@ D.65795, mod_reg_val
	ldreq	r0, [sp, #4]	@ D.65795, mod_reg_addr
	muleq	r0, r0, ip	@ D.65795, D.65795, D.65795
	streq	r0, [sp, #4]	@ D.65795, mod_reg_addr
	movw	r0, #819	@ tmp203,
	ldr	ip, [sp]	@ D.65795, mod_reg_val
	cmp	ip, r0	@ D.65795, tmp203
	beq	.L139	@,
	ldr	r0, [sp]	@ D.65795, mod_reg_val
	cmp	r0, #21504	@ D.65795,
	bne	.L134	@,
.L139:
	ldr	r0, [sp, #4]	@ D.65795, mod_reg_addr
	b	.L130	@
.L134:
	ldr	ip, [sp, #4]	@ D.65795, mod_reg_addr
	movw	r0, #770	@ tmp204,
	cmp	ip, r0	@ D.65795, tmp204
	ldrne	r0, [sp]	@ D.65795, mod_reg_val
	ldrne	ip, [sp, #4]	@ D.65795, mod_reg_addr
	addne	r0, r0, ip	@ D.65795, D.65795, D.65795
	strne	r0, [sp, #4]	@ D.65795, mod_reg_addr
	ldr	r0, [sp]	@ D.65795, mod_reg_val
	and	r0, r0, #243	@ D.65795, D.65795,
	str	r0, [sp, #4]	@ D.65795, mod_reg_addr
	movw	r0, #770	@ tmp205,
	ldr	ip, [sp, #4]	@ D.65795, mod_reg_addr
	cmp	ip, r0	@ D.65795, tmp205
	ldrne	r0, [sp]	@ D.65795, mod_reg_val
	ldrne	ip, [sp, #4]	@ D.65795, mod_reg_addr
	addne	r0, r0, ip	@ D.65795, D.65795, D.65795
	strne	r0, [sp, #4]	@ D.65795, mod_reg_addr
	movw	r0, #819	@ tmp206,
	ldr	ip, [sp, #4]	@ D.65795, mod_reg_addr
	mul	r0, r0, ip	@ D.65795, tmp206, D.65795
	str	r0, [sp]	@ D.65795, mod_reg_val
	ldr	r0, [sp]	@ D.65795, mod_reg_val
	and	r0, r0, #3072	@ D.65795, D.65795,
	str	r0, [sp, #4]	@ D.65795, mod_reg_addr
	ldr	r0, [sp]	@ D.65795, mod_reg_val
.L130:
	uxtb	r0, r3	@ errors, errors
	ubfx	r3, r3, #16, #8	@ D.65797, errors,,
	strh	r0, [r1]	@ movhi	@ errors, *t_err_9(D)
	strh	r3, [r2]	@ movhi	@ D.65797, *r_err_14(D)
	add	sp, sp, #16	@,,
	@ sp needed	@
	bx	lr	@
	.fnend
	.size	can_asm_clean_transfer_err, .-can_asm_clean_transfer_err
	.align	2
	.global	can_asm_start_xmit
	.syntax unified
	.arm
	.type	can_asm_start_xmit, %function
can_asm_start_xmit:
	.fnstart
	@ args = 8, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}	@
	.save {r4, r5, r6, r7, r8, r9, r10, fp, lr}
	tst	r2, #1073741824	@ id,
	ldr	r8, [r3]	@ dlc, *flag_7(D)
	mov	r6, r3	@ flag, flag
	.pad #20
	sub	sp, sp, #20	@,,
	mov	r5, r0	@ mod_reg_addr, mod_reg_addr
	mov	r4, r2	@ id, id
	ldr	r7, [sp, #56]	@ mod_reg_val, mod_reg_val
	orrne	r3, r8, #64	@ D.65816, dlc,
	ldr	r9, [sp, #60]	@ pdata, pdata
	strne	r3, [r6]	@ D.65816, *flag_7(D)
	cmp	r2, #0	@ id,
	bge	.L142	@,
	ldr	r3, [r6]	@ *flag_7(D), *flag_7(D)
	orr	r3, r3, #128	@ D.65816, *flag_7(D),
	str	r3, [r6]	@ D.65816, *flag_7(D)
	mov	r3, #84	@ tmp195,
	str	r3, [r7]	@ tmp195, *mod_reg_val_18(D)
	.syntax divided
@ 597 "drivers/net/can/sunxi_can_asm.c" 1
	dsb st
@ 0 "" 2
	.arm
	.syntax unified
	bl	arm_heavy_mb	@
	ubfx	r3, r4, #21, #8	@ D.65814, id,,
	.syntax divided
@ 95 "./arch/arm/include/asm/io.h" 1
	str r3, [r5, #68]	@ D.65814, MEM[(volatile u32 *)_25]
@ 0 "" 2
	.arm
	.syntax unified
	mov	r3, #5	@ tmp197,
	str	r3, [sp]	@ tmp197, mod_reg_val
	mov	r3, #11	@ tmp198,
	str	r3, [sp, #4]	@ tmp198, mod_reg_addr
	ldr	r2, [sp, #4]	@ D.65815, mod_reg_addr
	ldr	r3, [sp]	@ D.65815, mod_reg_val
	eor	r3, r3, r2	@ D.65815, D.65815, D.65815
	str	r3, [sp]	@ D.65815, mod_reg_val
	ldr	r3, [sp]	@ D.65815, mod_reg_val
	.syntax divided
@ 599 "drivers/net/can/sunxi_can_asm.c" 1
	dsb st
@ 0 "" 2
	.arm
	.syntax unified
	bl	arm_heavy_mb	@
	ubfx	r3, r4, #13, #8	@ D.65814, id,,
	.syntax divided
@ 95 "./arch/arm/include/asm/io.h" 1
	str r3, [r5, #72]	@ D.65814, MEM[(volatile u32 *)_31]
@ 0 "" 2
@ 600 "drivers/net/can/sunxi_can_asm.c" 1
	dsb st
@ 0 "" 2
	.arm
	.syntax unified
	bl	arm_heavy_mb	@
	ubfx	r3, r4, #5, #8	@ D.65814, id,,
	.syntax divided
@ 95 "./arch/arm/include/asm/io.h" 1
	str r3, [r5, #76]	@ D.65814, MEM[(volatile u32 *)_36]
@ 0 "" 2
@ 601 "drivers/net/can/sunxi_can_asm.c" 1
	dsb st
@ 0 "" 2
	.arm
	.syntax unified
	lsl	r4, r4, #3	@ D.65814, id,
	bl	arm_heavy_mb	@
	uxtb	r4, r4	@ D.65814, D.65814
	.syntax divided
@ 95 "./arch/arm/include/asm/io.h" 1
	str r4, [r5, #80]	@ D.65814, MEM[(volatile u32 *)_41]
@ 0 "" 2
	.arm
	.syntax unified
	b	.L143	@
.L142:
	mov	r3, #76	@ tmp202,
	str	r3, [r7]	@ tmp202, *mod_reg_val_18(D)
	.syntax divided
@ 604 "drivers/net/can/sunxi_can_asm.c" 1
	dsb st
@ 0 "" 2
	.arm
	.syntax unified
	bl	arm_heavy_mb	@
	ubfx	r3, r4, #3, #8	@ D.65814, id,,
	.syntax divided
@ 95 "./arch/arm/include/asm/io.h" 1
	str r3, [r5, #68]	@ D.65814, MEM[(volatile u32 *)_47]
@ 0 "" 2
@ 605 "drivers/net/can/sunxi_can_asm.c" 1
	dsb st
@ 0 "" 2
	.arm
	.syntax unified
	lsl	r4, r4, #5	@ D.65814, id,
	bl	arm_heavy_mb	@
	uxtb	r4, r4	@ D.65814, D.65814
	.syntax divided
@ 95 "./arch/arm/include/asm/io.h" 1
	str r4, [r5, #72]	@ D.65814, MEM[(volatile u32 *)_52]
@ 0 "" 2
	.arm
	.syntax unified
.L143:
	mov	r4, #0	@ i,
	mov	r10, #14	@ tmp210,
	mov	fp, #10	@ tmp211,
.L144:
	cmp	r4, r8	@ i, dlc
	beq	.L152	@,
	.syntax divided
@ 609 "drivers/net/can/sunxi_can_asm.c" 1
	dsb st
@ 0 "" 2
	.arm
	.syntax unified
	bl	arm_heavy_mb	@
	ldr	r3, [r7]	@ *mod_reg_val_18(D), *mod_reg_val_18(D)
	ldrb	r2, [r9, r4]	@ zero_extendqisi2	@ D.65814, MEM[base: pdata_56(D), index: _79, offset: 0B]
	add	r3, r3, r4, lsl #2	@ D.65814, *mod_reg_val_18(D), i,
	add	r3, r5, r3	@ D.65818, mod_reg_addr, D.65814
	.syntax divided
@ 95 "./arch/arm/include/asm/io.h" 1
	str r2, [r3]	@ D.65814, MEM[(volatile u32 *)_64]
@ 0 "" 2
	.arm
	.syntax unified
	cmp	r4, #0	@ i,
	bne	.L145	@,
	str	r10, [sp, #8]	@ tmp210, mod_reg_val
	str	fp, [sp, #12]	@ tmp211, mod_reg_addr
	ldr	r3, [sp, #12]	@ D.65815, mod_reg_addr
	cmp	r3, #3072	@ D.65815,
	ldr	r3, [sp, #8]	@ D.65815, mod_reg_val
	addne	r3, r3, #1	@ D.65815, D.65815,
	strne	r3, [sp, #8]	@ D.65815, mod_reg_val
	ldrne	r3, [sp, #8]	@ D.65815, mod_reg_val
.L145:
	add	r4, r4, #1	@ i, i,
	b	.L144	@
.L152:
	.syntax divided
@ 616 "drivers/net/can/sunxi_can_asm.c" 1
	dsb st
@ 0 "" 2
	.arm
	.syntax unified
	bl	arm_heavy_mb	@
	ldr	r3, [r6]	@ D.65814, *flag_7(D)
	.syntax divided
@ 95 "./arch/arm/include/asm/io.h" 1
	str r3, [r5, #64]	@ D.65814, MEM[(volatile u32 *)_70]
@ 0 "" 2
	.arm
	.syntax unified
	add	sp, sp, #20	@,,
	@ sp needed	@
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}	@
	.fnend
	.size	can_asm_start_xmit, .-can_asm_start_xmit
	.align	2
	.global	can_asm_rx
	.syntax unified
	.arm
	.type	can_asm_rx, %function
can_asm_rx:
	.fnstart
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r0, r1, r2, r4, r5, lr}	@
	.save {r4, r5, lr}
	.pad #12
	.syntax divided
@ 113 "./arch/arm/include/asm/io.h" 1
	ldr lr, [r0, #64]	@ __v, MEM[(volatile u32 *)_3]
@ 0 "" 2
@ 622 "drivers/net/can/sunxi_can_asm.c" 1
	dsb
@ 0 "" 2
	.arm
	.syntax unified
	tst	lr, #128	@ __v,
	uxtb	ip, lr	@ fi, __v
	beq	.L154	@,
	mov	lr, #84	@ tmp185,
	str	lr, [r2]	@ tmp185, *mod_reg_val_9(D)
	mov	r2, #9	@ tmp186,
	str	r2, [sp]	@ tmp186, mod_reg_val
	mov	r2, #16	@ tmp187,
	str	r2, [sp, #4]	@ tmp187, mod_reg_addr
	ldr	r2, [sp]	@ D.65829, mod_reg_val
	cmp	r2, #21504	@ D.65829,
	beq	.L167	@,
	ldr	lr, [sp, #4]	@ D.65829, mod_reg_addr
	movw	r2, #770	@ tmp188,
	cmp	lr, r2	@ D.65829, tmp188
	ldrne	r2, [sp]	@ D.65829, mod_reg_val
	ldrne	lr, [sp, #4]	@ D.65829, mod_reg_addr
	addne	r2, r2, lr	@ D.65829, D.65829, D.65829
	strne	r2, [sp, #4]	@ D.65829, mod_reg_addr
	ldr	r2, [sp]	@ D.65829, mod_reg_val
	cmp	r2, #3072	@ D.65829,
	beq	.L167	@,
	ldr	lr, [sp]	@ D.65829, mod_reg_val
	movw	r2, #819	@ tmp189,
	cmp	lr, r2	@ D.65829, tmp189
	beq	.L167	@,
	ldr	lr, [sp]	@ D.65829, mod_reg_val
	ldr	r2, [sp, #4]	@ D.65829, mod_reg_addr
	cmp	lr, r2	@ D.65829, D.65829
	bge	.L160	@,
	ldr	r2, [sp]	@ D.65829, mod_reg_val
	ldr	r4, .L169	@ tmp192,
	ldr	lr, [sp, #4]	@ D.65829, mod_reg_addr
	smull	r4, r5, r2, r4	@ tmp191, D.65829, tmp192
	sub	r2, r5, r2, asr #31	@ D.65829,, D.65829,
	add	r2, r2, lr	@ D.65829, D.65829, D.65829
	str	r2, [sp, #4]	@ D.65829, mod_reg_addr
.L160:
	ldr	lr, [sp, #4]	@ D.65829, mod_reg_addr
	ldr	r2, [sp]	@ D.65829, mod_reg_val
	eor	r2, r2, lr	@ D.65829, D.65829, D.65829
	str	r2, [sp]	@ D.65829, mod_reg_val
	ldr	r2, [sp, #4]	@ D.65829, mod_reg_addr
	str	r2, [sp]	@ D.65829, mod_reg_val
	ldr	r2, [sp]	@ D.65829, mod_reg_val
	and	r2, r2, #243	@ D.65829, D.65829,
	str	r2, [sp, #4]	@ D.65829, mod_reg_addr
	movw	r2, #770	@ tmp194,
	ldr	lr, [sp, #4]	@ D.65829, mod_reg_addr
	cmp	lr, r2	@ D.65829, tmp194
	ldrne	r2, [sp]	@ D.65829, mod_reg_val
	ldrne	lr, [sp, #4]	@ D.65829, mod_reg_addr
	addne	r2, r2, lr	@ D.65829, D.65829, D.65829
	strne	r2, [sp, #4]	@ D.65829, mod_reg_addr
	ldr	lr, [sp]	@ D.65829, mod_reg_val
	ldr	r2, [sp, #4]	@ D.65829, mod_reg_addr
	eor	r2, r2, lr	@ D.65829, D.65829, D.65829
	str	r2, [sp, #4]	@ D.65829, mod_reg_addr
	ldr	r2, [sp]	@ D.65829, mod_reg_val
	and	r2, r2, #243	@ D.65829, D.65829,
	str	r2, [sp, #4]	@ D.65829, mod_reg_addr
	movw	r2, #819	@ tmp195,
	ldr	lr, [sp]	@ D.65829, mod_reg_val
	cmp	lr, r2	@ D.65829, tmp195
	bne	.L162	@,
.L167:
	ldr	r2, [sp, #4]	@ D.65829, mod_reg_addr
	b	.L156	@
.L162:
	ldr	lr, [sp]	@ D.65829, mod_reg_val
	ldr	r2, [sp, #4]	@ D.65829, mod_reg_addr
	cmp	lr, r2	@ D.65829, D.65829
	bge	.L163	@,
	ldr	r2, [sp]	@ D.65829, mod_reg_val
	ldr	r4, .L169	@ tmp198,
	ldr	lr, [sp, #4]	@ D.65829, mod_reg_addr
	smull	r4, r5, r2, r4	@ tmp197, D.65829, tmp198
	sub	r2, r5, r2, asr #31	@ D.65829,, D.65829,
	add	r2, r2, lr	@ D.65829, D.65829, D.65829
	str	r2, [sp, #4]	@ D.65829, mod_reg_addr
.L163:
	ldr	r2, [sp]	@ D.65829, mod_reg_val
.L156:
	.syntax divided
@ 113 "./arch/arm/include/asm/io.h" 1
	ldr r4, [r0, #68]	@ __v, MEM[(volatile u32 *)_12]
@ 0 "" 2
@ 626 "drivers/net/can/sunxi_can_asm.c" 1
	dsb
@ 0 "" 2
@ 113 "./arch/arm/include/asm/io.h" 1
	ldr r2, [r0, #72]	@ __v, MEM[(volatile u32 *)_16]
@ 0 "" 2
@ 627 "drivers/net/can/sunxi_can_asm.c" 1
	dsb
@ 0 "" 2
@ 113 "./arch/arm/include/asm/io.h" 1
	ldr lr, [r0, #76]	@ __v, MEM[(volatile u32 *)_21]
@ 0 "" 2
@ 628 "drivers/net/can/sunxi_can_asm.c" 1
	dsb
@ 0 "" 2
@ 113 "./arch/arm/include/asm/io.h" 1
	ldr r0, [r0, #80]	@ __v, MEM[(volatile u32 *)_26]
@ 0 "" 2
@ 629 "drivers/net/can/sunxi_can_asm.c" 1
	dsb
@ 0 "" 2
	.arm
	.syntax unified
	lsl	r2, r2, #13	@ D.65828, __v,
	ubfx	r0, r0, #3, #5	@ D.65828, __v,,
	orr	r2, r2, r4, lsl #21	@ D.65828, D.65828, __v,
	orr	r2, r2, #-2147483648	@ D.65828, D.65828,
	orr	r2, r2, lr, lsl #5	@ D.65828, D.65828, __v,
	orr	r2, r2, r0	@ D.65828, D.65828, D.65828
	b	.L168	@
.L154:
	mov	lr, #76	@ tmp213,
	str	lr, [r2]	@ tmp213, *mod_reg_val_9(D)
	.syntax divided
@ 113 "./arch/arm/include/asm/io.h" 1
	ldr lr, [r0, #68]	@ __v, MEM[(volatile u32 *)_37]
@ 0 "" 2
@ 633 "drivers/net/can/sunxi_can_asm.c" 1
	dsb
@ 0 "" 2
@ 113 "./arch/arm/include/asm/io.h" 1
	ldr r2, [r0, #72]	@ __v, MEM[(volatile u32 *)_40]
@ 0 "" 2
@ 634 "drivers/net/can/sunxi_can_asm.c" 1
	dsb
@ 0 "" 2
	.arm
	.syntax unified
	ubfx	r2, r2, #5, #3	@ D.65828, __v,,
	orr	r2, r2, lr, lsl #3	@ D.65828, D.65828, __v,
.L168:
	str	r2, [r3]	@ D.65828, *id_32(D)
	strb	ip, [r1]	@ fi, *r_fi_47(D)
	add	sp, sp, #12	@,,
	@ sp needed	@
	pop	{r4, r5, pc}	@
.L170:
	.align	2
.L169:
	.word	1431655766
	.fnend
	.size	can_asm_rx, .-can_asm_rx
	.section	.rodata.str1.1,"aMS",%progbits,1
.LC0:
	.ascii	"reg\000"
	.ident	"GCC: (Linaro GCC 5.3-2016.05) 5.3.1 20160412"
	.section	.note.GNU-stack,"",%progbits
