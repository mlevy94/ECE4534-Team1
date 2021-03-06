#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-rover.mk)" "nbproject/Makefile-local-rover.mk"
include nbproject/Makefile-local-rover.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=rover
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/rover1.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/rover1.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=../src/system_config/rover/framework/driver/oc/src/drv_oc_static.c ../src/system_config/rover/framework/driver/tmr/src/drv_tmr_static.c ../src/system_config/rover/framework/driver/usart/drv_usart_static.c ../src/system_config/rover/framework/system/clk/src/sys_clk_static.c ../src/system_config/rover/framework/system/ports/src/sys_ports_static.c ../src/system_config/rover/system_init.c ../src/system_config/rover/system_interrupt.c ../src/system_config/rover/system_exceptions.c ../src/system_config/rover/system_tasks.c ../src/system_config/rover/system_interrupt_a.S ../src/system_config/rover/rtos_hooks.c ../src/motorapp.c ../src/nfc_app.c ../src/uart_tx_app.c ../src/uart_rx_app.c ../src/main.c ../src/comm.c ../src/debug.c ../../../../../../../../../microchip/harmony/v1_06_02/framework/osal/src/osal_freertos.c ../../../../../../../../../microchip/harmony/v1_06_02/framework/system/devcon/src/sys_devcon.c ../../../../../../../../../microchip/harmony/v1_06_02/framework/system/devcon/src/sys_devcon_pic32mx.c ../../../../../../../../../microchip/harmony/v1_06_02/framework/system/int/src/sys_int_pic32.c ../../../../../../../../../microchip/harmony/v1_06_02/framework/system/ports/src/sys_ports.c ../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MemMang/heap_4.c ../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX/port.c ../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX/port_asm.S ../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/croutine.c ../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/list.c ../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/queue.c ../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/tasks.c ../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/timers.c ../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/event_groups.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/743863841/drv_oc_static.o ${OBJECTDIR}/_ext/1096248328/drv_tmr_static.o ${OBJECTDIR}/_ext/1126308465/drv_usart_static.o ${OBJECTDIR}/_ext/950798984/sys_clk_static.o ${OBJECTDIR}/_ext/1692174648/sys_ports_static.o ${OBJECTDIR}/_ext/1433043121/system_init.o ${OBJECTDIR}/_ext/1433043121/system_interrupt.o ${OBJECTDIR}/_ext/1433043121/system_exceptions.o ${OBJECTDIR}/_ext/1433043121/system_tasks.o ${OBJECTDIR}/_ext/1433043121/system_interrupt_a.o ${OBJECTDIR}/_ext/1433043121/rtos_hooks.o ${OBJECTDIR}/_ext/1360937237/motorapp.o ${OBJECTDIR}/_ext/1360937237/nfc_app.o ${OBJECTDIR}/_ext/1360937237/uart_tx_app.o ${OBJECTDIR}/_ext/1360937237/uart_rx_app.o ${OBJECTDIR}/_ext/1360937237/main.o ${OBJECTDIR}/_ext/1360937237/comm.o ${OBJECTDIR}/_ext/1360937237/debug.o ${OBJECTDIR}/_ext/933768138/osal_freertos.o ${OBJECTDIR}/_ext/1502763280/sys_devcon.o ${OBJECTDIR}/_ext/1502763280/sys_devcon_pic32mx.o ${OBJECTDIR}/_ext/361851466/sys_int_pic32.o ${OBJECTDIR}/_ext/1967917095/sys_ports.o ${OBJECTDIR}/_ext/371772180/heap_4.o ${OBJECTDIR}/_ext/1767308377/port.o ${OBJECTDIR}/_ext/1767308377/port_asm.o ${OBJECTDIR}/_ext/1092921775/croutine.o ${OBJECTDIR}/_ext/1092921775/list.o ${OBJECTDIR}/_ext/1092921775/queue.o ${OBJECTDIR}/_ext/1092921775/tasks.o ${OBJECTDIR}/_ext/1092921775/timers.o ${OBJECTDIR}/_ext/1092921775/event_groups.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/743863841/drv_oc_static.o.d ${OBJECTDIR}/_ext/1096248328/drv_tmr_static.o.d ${OBJECTDIR}/_ext/1126308465/drv_usart_static.o.d ${OBJECTDIR}/_ext/950798984/sys_clk_static.o.d ${OBJECTDIR}/_ext/1692174648/sys_ports_static.o.d ${OBJECTDIR}/_ext/1433043121/system_init.o.d ${OBJECTDIR}/_ext/1433043121/system_interrupt.o.d ${OBJECTDIR}/_ext/1433043121/system_exceptions.o.d ${OBJECTDIR}/_ext/1433043121/system_tasks.o.d ${OBJECTDIR}/_ext/1433043121/system_interrupt_a.o.d ${OBJECTDIR}/_ext/1433043121/rtos_hooks.o.d ${OBJECTDIR}/_ext/1360937237/motorapp.o.d ${OBJECTDIR}/_ext/1360937237/nfc_app.o.d ${OBJECTDIR}/_ext/1360937237/uart_tx_app.o.d ${OBJECTDIR}/_ext/1360937237/uart_rx_app.o.d ${OBJECTDIR}/_ext/1360937237/main.o.d ${OBJECTDIR}/_ext/1360937237/comm.o.d ${OBJECTDIR}/_ext/1360937237/debug.o.d ${OBJECTDIR}/_ext/933768138/osal_freertos.o.d ${OBJECTDIR}/_ext/1502763280/sys_devcon.o.d ${OBJECTDIR}/_ext/1502763280/sys_devcon_pic32mx.o.d ${OBJECTDIR}/_ext/361851466/sys_int_pic32.o.d ${OBJECTDIR}/_ext/1967917095/sys_ports.o.d ${OBJECTDIR}/_ext/371772180/heap_4.o.d ${OBJECTDIR}/_ext/1767308377/port.o.d ${OBJECTDIR}/_ext/1767308377/port_asm.o.d ${OBJECTDIR}/_ext/1092921775/croutine.o.d ${OBJECTDIR}/_ext/1092921775/list.o.d ${OBJECTDIR}/_ext/1092921775/queue.o.d ${OBJECTDIR}/_ext/1092921775/tasks.o.d ${OBJECTDIR}/_ext/1092921775/timers.o.d ${OBJECTDIR}/_ext/1092921775/event_groups.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/743863841/drv_oc_static.o ${OBJECTDIR}/_ext/1096248328/drv_tmr_static.o ${OBJECTDIR}/_ext/1126308465/drv_usart_static.o ${OBJECTDIR}/_ext/950798984/sys_clk_static.o ${OBJECTDIR}/_ext/1692174648/sys_ports_static.o ${OBJECTDIR}/_ext/1433043121/system_init.o ${OBJECTDIR}/_ext/1433043121/system_interrupt.o ${OBJECTDIR}/_ext/1433043121/system_exceptions.o ${OBJECTDIR}/_ext/1433043121/system_tasks.o ${OBJECTDIR}/_ext/1433043121/system_interrupt_a.o ${OBJECTDIR}/_ext/1433043121/rtos_hooks.o ${OBJECTDIR}/_ext/1360937237/motorapp.o ${OBJECTDIR}/_ext/1360937237/nfc_app.o ${OBJECTDIR}/_ext/1360937237/uart_tx_app.o ${OBJECTDIR}/_ext/1360937237/uart_rx_app.o ${OBJECTDIR}/_ext/1360937237/main.o ${OBJECTDIR}/_ext/1360937237/comm.o ${OBJECTDIR}/_ext/1360937237/debug.o ${OBJECTDIR}/_ext/933768138/osal_freertos.o ${OBJECTDIR}/_ext/1502763280/sys_devcon.o ${OBJECTDIR}/_ext/1502763280/sys_devcon_pic32mx.o ${OBJECTDIR}/_ext/361851466/sys_int_pic32.o ${OBJECTDIR}/_ext/1967917095/sys_ports.o ${OBJECTDIR}/_ext/371772180/heap_4.o ${OBJECTDIR}/_ext/1767308377/port.o ${OBJECTDIR}/_ext/1767308377/port_asm.o ${OBJECTDIR}/_ext/1092921775/croutine.o ${OBJECTDIR}/_ext/1092921775/list.o ${OBJECTDIR}/_ext/1092921775/queue.o ${OBJECTDIR}/_ext/1092921775/tasks.o ${OBJECTDIR}/_ext/1092921775/timers.o ${OBJECTDIR}/_ext/1092921775/event_groups.o

# Source Files
SOURCEFILES=../src/system_config/rover/framework/driver/oc/src/drv_oc_static.c ../src/system_config/rover/framework/driver/tmr/src/drv_tmr_static.c ../src/system_config/rover/framework/driver/usart/drv_usart_static.c ../src/system_config/rover/framework/system/clk/src/sys_clk_static.c ../src/system_config/rover/framework/system/ports/src/sys_ports_static.c ../src/system_config/rover/system_init.c ../src/system_config/rover/system_interrupt.c ../src/system_config/rover/system_exceptions.c ../src/system_config/rover/system_tasks.c ../src/system_config/rover/system_interrupt_a.S ../src/system_config/rover/rtos_hooks.c ../src/motorapp.c ../src/nfc_app.c ../src/uart_tx_app.c ../src/uart_rx_app.c ../src/main.c ../src/comm.c ../src/debug.c ../../../../../../../../../microchip/harmony/v1_06_02/framework/osal/src/osal_freertos.c ../../../../../../../../../microchip/harmony/v1_06_02/framework/system/devcon/src/sys_devcon.c ../../../../../../../../../microchip/harmony/v1_06_02/framework/system/devcon/src/sys_devcon_pic32mx.c ../../../../../../../../../microchip/harmony/v1_06_02/framework/system/int/src/sys_int_pic32.c ../../../../../../../../../microchip/harmony/v1_06_02/framework/system/ports/src/sys_ports.c ../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MemMang/heap_4.c ../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX/port.c ../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX/port_asm.S ../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/croutine.c ../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/list.c ../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/queue.c ../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/tasks.c ../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/timers.c ../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/event_groups.c


CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-rover.mk dist/${CND_CONF}/${IMAGE_TYPE}/rover1.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=32MX795F512L
MP_LINKER_FILE_OPTION=
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assembleWithPreprocess
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/1433043121/system_interrupt_a.o: ../src/system_config/rover/system_interrupt_a.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1433043121" 
	@${RM} ${OBJECTDIR}/_ext/1433043121/system_interrupt_a.o.d 
	@${RM} ${OBJECTDIR}/_ext/1433043121/system_interrupt_a.o 
	@${RM} ${OBJECTDIR}/_ext/1433043121/system_interrupt_a.o.ok ${OBJECTDIR}/_ext/1433043121/system_interrupt_a.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1433043121/system_interrupt_a.o.d" "${OBJECTDIR}/_ext/1433043121/system_interrupt_a.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC} $(MP_EXTRA_AS_PRE)  -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -I"../src/system_config/rover" -MMD -MF "${OBJECTDIR}/_ext/1433043121/system_interrupt_a.o.d"  -o ${OBJECTDIR}/_ext/1433043121/system_interrupt_a.o ../src/system_config/rover/system_interrupt_a.S    -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/1433043121/system_interrupt_a.o.asm.d",--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--gdwarf-2,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_PK3=1
	
${OBJECTDIR}/_ext/1767308377/port_asm.o: ../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX/port_asm.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1767308377" 
	@${RM} ${OBJECTDIR}/_ext/1767308377/port_asm.o.d 
	@${RM} ${OBJECTDIR}/_ext/1767308377/port_asm.o 
	@${RM} ${OBJECTDIR}/_ext/1767308377/port_asm.o.ok ${OBJECTDIR}/_ext/1767308377/port_asm.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1767308377/port_asm.o.d" "${OBJECTDIR}/_ext/1767308377/port_asm.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC} $(MP_EXTRA_AS_PRE)  -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -I"../src/system_config/rover" -MMD -MF "${OBJECTDIR}/_ext/1767308377/port_asm.o.d"  -o ${OBJECTDIR}/_ext/1767308377/port_asm.o ../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX/port_asm.S    -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/1767308377/port_asm.o.asm.d",--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--gdwarf-2,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_PK3=1
	
else
${OBJECTDIR}/_ext/1433043121/system_interrupt_a.o: ../src/system_config/rover/system_interrupt_a.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1433043121" 
	@${RM} ${OBJECTDIR}/_ext/1433043121/system_interrupt_a.o.d 
	@${RM} ${OBJECTDIR}/_ext/1433043121/system_interrupt_a.o 
	@${RM} ${OBJECTDIR}/_ext/1433043121/system_interrupt_a.o.ok ${OBJECTDIR}/_ext/1433043121/system_interrupt_a.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1433043121/system_interrupt_a.o.d" "${OBJECTDIR}/_ext/1433043121/system_interrupt_a.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC} $(MP_EXTRA_AS_PRE)  -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -I"../src/system_config/rover" -MMD -MF "${OBJECTDIR}/_ext/1433043121/system_interrupt_a.o.d"  -o ${OBJECTDIR}/_ext/1433043121/system_interrupt_a.o ../src/system_config/rover/system_interrupt_a.S    -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/1433043121/system_interrupt_a.o.asm.d",--gdwarf-2
	
${OBJECTDIR}/_ext/1767308377/port_asm.o: ../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX/port_asm.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1767308377" 
	@${RM} ${OBJECTDIR}/_ext/1767308377/port_asm.o.d 
	@${RM} ${OBJECTDIR}/_ext/1767308377/port_asm.o 
	@${RM} ${OBJECTDIR}/_ext/1767308377/port_asm.o.ok ${OBJECTDIR}/_ext/1767308377/port_asm.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1767308377/port_asm.o.d" "${OBJECTDIR}/_ext/1767308377/port_asm.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC} $(MP_EXTRA_AS_PRE)  -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -I"../src/system_config/rover" -MMD -MF "${OBJECTDIR}/_ext/1767308377/port_asm.o.d"  -o ${OBJECTDIR}/_ext/1767308377/port_asm.o ../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX/port_asm.S    -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/1767308377/port_asm.o.asm.d",--gdwarf-2
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/743863841/drv_oc_static.o: ../src/system_config/rover/framework/driver/oc/src/drv_oc_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/743863841" 
	@${RM} ${OBJECTDIR}/_ext/743863841/drv_oc_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/743863841/drv_oc_static.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/743863841/drv_oc_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/743863841/drv_oc_static.o.d" -o ${OBJECTDIR}/_ext/743863841/drv_oc_static.o ../src/system_config/rover/framework/driver/oc/src/drv_oc_static.c     
	
${OBJECTDIR}/_ext/1096248328/drv_tmr_static.o: ../src/system_config/rover/framework/driver/tmr/src/drv_tmr_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1096248328" 
	@${RM} ${OBJECTDIR}/_ext/1096248328/drv_tmr_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/1096248328/drv_tmr_static.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1096248328/drv_tmr_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1096248328/drv_tmr_static.o.d" -o ${OBJECTDIR}/_ext/1096248328/drv_tmr_static.o ../src/system_config/rover/framework/driver/tmr/src/drv_tmr_static.c     
	
${OBJECTDIR}/_ext/1126308465/drv_usart_static.o: ../src/system_config/rover/framework/driver/usart/drv_usart_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1126308465" 
	@${RM} ${OBJECTDIR}/_ext/1126308465/drv_usart_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/1126308465/drv_usart_static.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1126308465/drv_usart_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1126308465/drv_usart_static.o.d" -o ${OBJECTDIR}/_ext/1126308465/drv_usart_static.o ../src/system_config/rover/framework/driver/usart/drv_usart_static.c     
	
${OBJECTDIR}/_ext/950798984/sys_clk_static.o: ../src/system_config/rover/framework/system/clk/src/sys_clk_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/950798984" 
	@${RM} ${OBJECTDIR}/_ext/950798984/sys_clk_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/950798984/sys_clk_static.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/950798984/sys_clk_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/950798984/sys_clk_static.o.d" -o ${OBJECTDIR}/_ext/950798984/sys_clk_static.o ../src/system_config/rover/framework/system/clk/src/sys_clk_static.c     
	
${OBJECTDIR}/_ext/1692174648/sys_ports_static.o: ../src/system_config/rover/framework/system/ports/src/sys_ports_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1692174648" 
	@${RM} ${OBJECTDIR}/_ext/1692174648/sys_ports_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/1692174648/sys_ports_static.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1692174648/sys_ports_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1692174648/sys_ports_static.o.d" -o ${OBJECTDIR}/_ext/1692174648/sys_ports_static.o ../src/system_config/rover/framework/system/ports/src/sys_ports_static.c     
	
${OBJECTDIR}/_ext/1433043121/system_init.o: ../src/system_config/rover/system_init.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1433043121" 
	@${RM} ${OBJECTDIR}/_ext/1433043121/system_init.o.d 
	@${RM} ${OBJECTDIR}/_ext/1433043121/system_init.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1433043121/system_init.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1433043121/system_init.o.d" -o ${OBJECTDIR}/_ext/1433043121/system_init.o ../src/system_config/rover/system_init.c     
	
${OBJECTDIR}/_ext/1433043121/system_interrupt.o: ../src/system_config/rover/system_interrupt.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1433043121" 
	@${RM} ${OBJECTDIR}/_ext/1433043121/system_interrupt.o.d 
	@${RM} ${OBJECTDIR}/_ext/1433043121/system_interrupt.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1433043121/system_interrupt.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1433043121/system_interrupt.o.d" -o ${OBJECTDIR}/_ext/1433043121/system_interrupt.o ../src/system_config/rover/system_interrupt.c     
	
${OBJECTDIR}/_ext/1433043121/system_exceptions.o: ../src/system_config/rover/system_exceptions.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1433043121" 
	@${RM} ${OBJECTDIR}/_ext/1433043121/system_exceptions.o.d 
	@${RM} ${OBJECTDIR}/_ext/1433043121/system_exceptions.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1433043121/system_exceptions.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1433043121/system_exceptions.o.d" -o ${OBJECTDIR}/_ext/1433043121/system_exceptions.o ../src/system_config/rover/system_exceptions.c     
	
${OBJECTDIR}/_ext/1433043121/system_tasks.o: ../src/system_config/rover/system_tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1433043121" 
	@${RM} ${OBJECTDIR}/_ext/1433043121/system_tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1433043121/system_tasks.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1433043121/system_tasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1433043121/system_tasks.o.d" -o ${OBJECTDIR}/_ext/1433043121/system_tasks.o ../src/system_config/rover/system_tasks.c     
	
${OBJECTDIR}/_ext/1433043121/rtos_hooks.o: ../src/system_config/rover/rtos_hooks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1433043121" 
	@${RM} ${OBJECTDIR}/_ext/1433043121/rtos_hooks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1433043121/rtos_hooks.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1433043121/rtos_hooks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1433043121/rtos_hooks.o.d" -o ${OBJECTDIR}/_ext/1433043121/rtos_hooks.o ../src/system_config/rover/rtos_hooks.c     
	
${OBJECTDIR}/_ext/1360937237/motorapp.o: ../src/motorapp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/motorapp.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/motorapp.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/motorapp.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1360937237/motorapp.o.d" -o ${OBJECTDIR}/_ext/1360937237/motorapp.o ../src/motorapp.c     
	
${OBJECTDIR}/_ext/1360937237/nfc_app.o: ../src/nfc_app.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/nfc_app.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/nfc_app.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/nfc_app.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1360937237/nfc_app.o.d" -o ${OBJECTDIR}/_ext/1360937237/nfc_app.o ../src/nfc_app.c     
	
${OBJECTDIR}/_ext/1360937237/uart_tx_app.o: ../src/uart_tx_app.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/uart_tx_app.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/uart_tx_app.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/uart_tx_app.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1360937237/uart_tx_app.o.d" -o ${OBJECTDIR}/_ext/1360937237/uart_tx_app.o ../src/uart_tx_app.c     
	
${OBJECTDIR}/_ext/1360937237/uart_rx_app.o: ../src/uart_rx_app.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/uart_rx_app.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/uart_rx_app.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/uart_rx_app.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1360937237/uart_rx_app.o.d" -o ${OBJECTDIR}/_ext/1360937237/uart_rx_app.o ../src/uart_rx_app.c     
	
${OBJECTDIR}/_ext/1360937237/main.o: ../src/main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1360937237/main.o.d" -o ${OBJECTDIR}/_ext/1360937237/main.o ../src/main.c     
	
${OBJECTDIR}/_ext/1360937237/comm.o: ../src/comm.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/comm.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/comm.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/comm.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1360937237/comm.o.d" -o ${OBJECTDIR}/_ext/1360937237/comm.o ../src/comm.c     
	
${OBJECTDIR}/_ext/1360937237/debug.o: ../src/debug.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/debug.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/debug.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/debug.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1360937237/debug.o.d" -o ${OBJECTDIR}/_ext/1360937237/debug.o ../src/debug.c     
	
${OBJECTDIR}/_ext/933768138/osal_freertos.o: ../../../../../../../../../microchip/harmony/v1_06_02/framework/osal/src/osal_freertos.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/933768138" 
	@${RM} ${OBJECTDIR}/_ext/933768138/osal_freertos.o.d 
	@${RM} ${OBJECTDIR}/_ext/933768138/osal_freertos.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/933768138/osal_freertos.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/933768138/osal_freertos.o.d" -o ${OBJECTDIR}/_ext/933768138/osal_freertos.o ../../../../../../../../../microchip/harmony/v1_06_02/framework/osal/src/osal_freertos.c     
	
${OBJECTDIR}/_ext/1502763280/sys_devcon.o: ../../../../../../../../../microchip/harmony/v1_06_02/framework/system/devcon/src/sys_devcon.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1502763280" 
	@${RM} ${OBJECTDIR}/_ext/1502763280/sys_devcon.o.d 
	@${RM} ${OBJECTDIR}/_ext/1502763280/sys_devcon.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1502763280/sys_devcon.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1502763280/sys_devcon.o.d" -o ${OBJECTDIR}/_ext/1502763280/sys_devcon.o ../../../../../../../../../microchip/harmony/v1_06_02/framework/system/devcon/src/sys_devcon.c     
	
${OBJECTDIR}/_ext/1502763280/sys_devcon_pic32mx.o: ../../../../../../../../../microchip/harmony/v1_06_02/framework/system/devcon/src/sys_devcon_pic32mx.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1502763280" 
	@${RM} ${OBJECTDIR}/_ext/1502763280/sys_devcon_pic32mx.o.d 
	@${RM} ${OBJECTDIR}/_ext/1502763280/sys_devcon_pic32mx.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1502763280/sys_devcon_pic32mx.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1502763280/sys_devcon_pic32mx.o.d" -o ${OBJECTDIR}/_ext/1502763280/sys_devcon_pic32mx.o ../../../../../../../../../microchip/harmony/v1_06_02/framework/system/devcon/src/sys_devcon_pic32mx.c     
	
${OBJECTDIR}/_ext/361851466/sys_int_pic32.o: ../../../../../../../../../microchip/harmony/v1_06_02/framework/system/int/src/sys_int_pic32.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/361851466" 
	@${RM} ${OBJECTDIR}/_ext/361851466/sys_int_pic32.o.d 
	@${RM} ${OBJECTDIR}/_ext/361851466/sys_int_pic32.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/361851466/sys_int_pic32.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/361851466/sys_int_pic32.o.d" -o ${OBJECTDIR}/_ext/361851466/sys_int_pic32.o ../../../../../../../../../microchip/harmony/v1_06_02/framework/system/int/src/sys_int_pic32.c     
	
${OBJECTDIR}/_ext/1967917095/sys_ports.o: ../../../../../../../../../microchip/harmony/v1_06_02/framework/system/ports/src/sys_ports.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1967917095" 
	@${RM} ${OBJECTDIR}/_ext/1967917095/sys_ports.o.d 
	@${RM} ${OBJECTDIR}/_ext/1967917095/sys_ports.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1967917095/sys_ports.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1967917095/sys_ports.o.d" -o ${OBJECTDIR}/_ext/1967917095/sys_ports.o ../../../../../../../../../microchip/harmony/v1_06_02/framework/system/ports/src/sys_ports.c     
	
${OBJECTDIR}/_ext/371772180/heap_4.o: ../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MemMang/heap_4.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/371772180" 
	@${RM} ${OBJECTDIR}/_ext/371772180/heap_4.o.d 
	@${RM} ${OBJECTDIR}/_ext/371772180/heap_4.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/371772180/heap_4.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/371772180/heap_4.o.d" -o ${OBJECTDIR}/_ext/371772180/heap_4.o ../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MemMang/heap_4.c     
	
${OBJECTDIR}/_ext/1767308377/port.o: ../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX/port.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1767308377" 
	@${RM} ${OBJECTDIR}/_ext/1767308377/port.o.d 
	@${RM} ${OBJECTDIR}/_ext/1767308377/port.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1767308377/port.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1767308377/port.o.d" -o ${OBJECTDIR}/_ext/1767308377/port.o ../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX/port.c     
	
${OBJECTDIR}/_ext/1092921775/croutine.o: ../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/croutine.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1092921775" 
	@${RM} ${OBJECTDIR}/_ext/1092921775/croutine.o.d 
	@${RM} ${OBJECTDIR}/_ext/1092921775/croutine.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1092921775/croutine.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1092921775/croutine.o.d" -o ${OBJECTDIR}/_ext/1092921775/croutine.o ../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/croutine.c     
	
${OBJECTDIR}/_ext/1092921775/list.o: ../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/list.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1092921775" 
	@${RM} ${OBJECTDIR}/_ext/1092921775/list.o.d 
	@${RM} ${OBJECTDIR}/_ext/1092921775/list.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1092921775/list.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1092921775/list.o.d" -o ${OBJECTDIR}/_ext/1092921775/list.o ../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/list.c     
	
${OBJECTDIR}/_ext/1092921775/queue.o: ../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/queue.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1092921775" 
	@${RM} ${OBJECTDIR}/_ext/1092921775/queue.o.d 
	@${RM} ${OBJECTDIR}/_ext/1092921775/queue.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1092921775/queue.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1092921775/queue.o.d" -o ${OBJECTDIR}/_ext/1092921775/queue.o ../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/queue.c     
	
${OBJECTDIR}/_ext/1092921775/tasks.o: ../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1092921775" 
	@${RM} ${OBJECTDIR}/_ext/1092921775/tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1092921775/tasks.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1092921775/tasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1092921775/tasks.o.d" -o ${OBJECTDIR}/_ext/1092921775/tasks.o ../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/tasks.c     
	
${OBJECTDIR}/_ext/1092921775/timers.o: ../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/timers.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1092921775" 
	@${RM} ${OBJECTDIR}/_ext/1092921775/timers.o.d 
	@${RM} ${OBJECTDIR}/_ext/1092921775/timers.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1092921775/timers.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1092921775/timers.o.d" -o ${OBJECTDIR}/_ext/1092921775/timers.o ../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/timers.c     
	
${OBJECTDIR}/_ext/1092921775/event_groups.o: ../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/event_groups.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1092921775" 
	@${RM} ${OBJECTDIR}/_ext/1092921775/event_groups.o.d 
	@${RM} ${OBJECTDIR}/_ext/1092921775/event_groups.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1092921775/event_groups.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1092921775/event_groups.o.d" -o ${OBJECTDIR}/_ext/1092921775/event_groups.o ../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/event_groups.c     
	
else
${OBJECTDIR}/_ext/743863841/drv_oc_static.o: ../src/system_config/rover/framework/driver/oc/src/drv_oc_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/743863841" 
	@${RM} ${OBJECTDIR}/_ext/743863841/drv_oc_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/743863841/drv_oc_static.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/743863841/drv_oc_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/743863841/drv_oc_static.o.d" -o ${OBJECTDIR}/_ext/743863841/drv_oc_static.o ../src/system_config/rover/framework/driver/oc/src/drv_oc_static.c     
	
${OBJECTDIR}/_ext/1096248328/drv_tmr_static.o: ../src/system_config/rover/framework/driver/tmr/src/drv_tmr_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1096248328" 
	@${RM} ${OBJECTDIR}/_ext/1096248328/drv_tmr_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/1096248328/drv_tmr_static.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1096248328/drv_tmr_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1096248328/drv_tmr_static.o.d" -o ${OBJECTDIR}/_ext/1096248328/drv_tmr_static.o ../src/system_config/rover/framework/driver/tmr/src/drv_tmr_static.c     
	
${OBJECTDIR}/_ext/1126308465/drv_usart_static.o: ../src/system_config/rover/framework/driver/usart/drv_usart_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1126308465" 
	@${RM} ${OBJECTDIR}/_ext/1126308465/drv_usart_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/1126308465/drv_usart_static.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1126308465/drv_usart_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1126308465/drv_usart_static.o.d" -o ${OBJECTDIR}/_ext/1126308465/drv_usart_static.o ../src/system_config/rover/framework/driver/usart/drv_usart_static.c     
	
${OBJECTDIR}/_ext/950798984/sys_clk_static.o: ../src/system_config/rover/framework/system/clk/src/sys_clk_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/950798984" 
	@${RM} ${OBJECTDIR}/_ext/950798984/sys_clk_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/950798984/sys_clk_static.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/950798984/sys_clk_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/950798984/sys_clk_static.o.d" -o ${OBJECTDIR}/_ext/950798984/sys_clk_static.o ../src/system_config/rover/framework/system/clk/src/sys_clk_static.c     
	
${OBJECTDIR}/_ext/1692174648/sys_ports_static.o: ../src/system_config/rover/framework/system/ports/src/sys_ports_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1692174648" 
	@${RM} ${OBJECTDIR}/_ext/1692174648/sys_ports_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/1692174648/sys_ports_static.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1692174648/sys_ports_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1692174648/sys_ports_static.o.d" -o ${OBJECTDIR}/_ext/1692174648/sys_ports_static.o ../src/system_config/rover/framework/system/ports/src/sys_ports_static.c     
	
${OBJECTDIR}/_ext/1433043121/system_init.o: ../src/system_config/rover/system_init.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1433043121" 
	@${RM} ${OBJECTDIR}/_ext/1433043121/system_init.o.d 
	@${RM} ${OBJECTDIR}/_ext/1433043121/system_init.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1433043121/system_init.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1433043121/system_init.o.d" -o ${OBJECTDIR}/_ext/1433043121/system_init.o ../src/system_config/rover/system_init.c     
	
${OBJECTDIR}/_ext/1433043121/system_interrupt.o: ../src/system_config/rover/system_interrupt.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1433043121" 
	@${RM} ${OBJECTDIR}/_ext/1433043121/system_interrupt.o.d 
	@${RM} ${OBJECTDIR}/_ext/1433043121/system_interrupt.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1433043121/system_interrupt.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1433043121/system_interrupt.o.d" -o ${OBJECTDIR}/_ext/1433043121/system_interrupt.o ../src/system_config/rover/system_interrupt.c     
	
${OBJECTDIR}/_ext/1433043121/system_exceptions.o: ../src/system_config/rover/system_exceptions.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1433043121" 
	@${RM} ${OBJECTDIR}/_ext/1433043121/system_exceptions.o.d 
	@${RM} ${OBJECTDIR}/_ext/1433043121/system_exceptions.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1433043121/system_exceptions.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1433043121/system_exceptions.o.d" -o ${OBJECTDIR}/_ext/1433043121/system_exceptions.o ../src/system_config/rover/system_exceptions.c     
	
${OBJECTDIR}/_ext/1433043121/system_tasks.o: ../src/system_config/rover/system_tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1433043121" 
	@${RM} ${OBJECTDIR}/_ext/1433043121/system_tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1433043121/system_tasks.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1433043121/system_tasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1433043121/system_tasks.o.d" -o ${OBJECTDIR}/_ext/1433043121/system_tasks.o ../src/system_config/rover/system_tasks.c     
	
${OBJECTDIR}/_ext/1433043121/rtos_hooks.o: ../src/system_config/rover/rtos_hooks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1433043121" 
	@${RM} ${OBJECTDIR}/_ext/1433043121/rtos_hooks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1433043121/rtos_hooks.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1433043121/rtos_hooks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1433043121/rtos_hooks.o.d" -o ${OBJECTDIR}/_ext/1433043121/rtos_hooks.o ../src/system_config/rover/rtos_hooks.c     
	
${OBJECTDIR}/_ext/1360937237/motorapp.o: ../src/motorapp.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/motorapp.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/motorapp.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/motorapp.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1360937237/motorapp.o.d" -o ${OBJECTDIR}/_ext/1360937237/motorapp.o ../src/motorapp.c     
	
${OBJECTDIR}/_ext/1360937237/nfc_app.o: ../src/nfc_app.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/nfc_app.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/nfc_app.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/nfc_app.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1360937237/nfc_app.o.d" -o ${OBJECTDIR}/_ext/1360937237/nfc_app.o ../src/nfc_app.c     
	
${OBJECTDIR}/_ext/1360937237/uart_tx_app.o: ../src/uart_tx_app.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/uart_tx_app.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/uart_tx_app.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/uart_tx_app.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1360937237/uart_tx_app.o.d" -o ${OBJECTDIR}/_ext/1360937237/uart_tx_app.o ../src/uart_tx_app.c     
	
${OBJECTDIR}/_ext/1360937237/uart_rx_app.o: ../src/uart_rx_app.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/uart_rx_app.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/uart_rx_app.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/uart_rx_app.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1360937237/uart_rx_app.o.d" -o ${OBJECTDIR}/_ext/1360937237/uart_rx_app.o ../src/uart_rx_app.c     
	
${OBJECTDIR}/_ext/1360937237/main.o: ../src/main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1360937237/main.o.d" -o ${OBJECTDIR}/_ext/1360937237/main.o ../src/main.c     
	
${OBJECTDIR}/_ext/1360937237/comm.o: ../src/comm.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/comm.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/comm.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/comm.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1360937237/comm.o.d" -o ${OBJECTDIR}/_ext/1360937237/comm.o ../src/comm.c     
	
${OBJECTDIR}/_ext/1360937237/debug.o: ../src/debug.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/debug.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/debug.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/debug.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1360937237/debug.o.d" -o ${OBJECTDIR}/_ext/1360937237/debug.o ../src/debug.c     
	
${OBJECTDIR}/_ext/933768138/osal_freertos.o: ../../../../../../../../../microchip/harmony/v1_06_02/framework/osal/src/osal_freertos.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/933768138" 
	@${RM} ${OBJECTDIR}/_ext/933768138/osal_freertos.o.d 
	@${RM} ${OBJECTDIR}/_ext/933768138/osal_freertos.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/933768138/osal_freertos.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/933768138/osal_freertos.o.d" -o ${OBJECTDIR}/_ext/933768138/osal_freertos.o ../../../../../../../../../microchip/harmony/v1_06_02/framework/osal/src/osal_freertos.c     
	
${OBJECTDIR}/_ext/1502763280/sys_devcon.o: ../../../../../../../../../microchip/harmony/v1_06_02/framework/system/devcon/src/sys_devcon.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1502763280" 
	@${RM} ${OBJECTDIR}/_ext/1502763280/sys_devcon.o.d 
	@${RM} ${OBJECTDIR}/_ext/1502763280/sys_devcon.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1502763280/sys_devcon.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1502763280/sys_devcon.o.d" -o ${OBJECTDIR}/_ext/1502763280/sys_devcon.o ../../../../../../../../../microchip/harmony/v1_06_02/framework/system/devcon/src/sys_devcon.c     
	
${OBJECTDIR}/_ext/1502763280/sys_devcon_pic32mx.o: ../../../../../../../../../microchip/harmony/v1_06_02/framework/system/devcon/src/sys_devcon_pic32mx.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1502763280" 
	@${RM} ${OBJECTDIR}/_ext/1502763280/sys_devcon_pic32mx.o.d 
	@${RM} ${OBJECTDIR}/_ext/1502763280/sys_devcon_pic32mx.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1502763280/sys_devcon_pic32mx.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1502763280/sys_devcon_pic32mx.o.d" -o ${OBJECTDIR}/_ext/1502763280/sys_devcon_pic32mx.o ../../../../../../../../../microchip/harmony/v1_06_02/framework/system/devcon/src/sys_devcon_pic32mx.c     
	
${OBJECTDIR}/_ext/361851466/sys_int_pic32.o: ../../../../../../../../../microchip/harmony/v1_06_02/framework/system/int/src/sys_int_pic32.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/361851466" 
	@${RM} ${OBJECTDIR}/_ext/361851466/sys_int_pic32.o.d 
	@${RM} ${OBJECTDIR}/_ext/361851466/sys_int_pic32.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/361851466/sys_int_pic32.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/361851466/sys_int_pic32.o.d" -o ${OBJECTDIR}/_ext/361851466/sys_int_pic32.o ../../../../../../../../../microchip/harmony/v1_06_02/framework/system/int/src/sys_int_pic32.c     
	
${OBJECTDIR}/_ext/1967917095/sys_ports.o: ../../../../../../../../../microchip/harmony/v1_06_02/framework/system/ports/src/sys_ports.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1967917095" 
	@${RM} ${OBJECTDIR}/_ext/1967917095/sys_ports.o.d 
	@${RM} ${OBJECTDIR}/_ext/1967917095/sys_ports.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1967917095/sys_ports.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1967917095/sys_ports.o.d" -o ${OBJECTDIR}/_ext/1967917095/sys_ports.o ../../../../../../../../../microchip/harmony/v1_06_02/framework/system/ports/src/sys_ports.c     
	
${OBJECTDIR}/_ext/371772180/heap_4.o: ../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MemMang/heap_4.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/371772180" 
	@${RM} ${OBJECTDIR}/_ext/371772180/heap_4.o.d 
	@${RM} ${OBJECTDIR}/_ext/371772180/heap_4.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/371772180/heap_4.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/371772180/heap_4.o.d" -o ${OBJECTDIR}/_ext/371772180/heap_4.o ../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MemMang/heap_4.c     
	
${OBJECTDIR}/_ext/1767308377/port.o: ../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX/port.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1767308377" 
	@${RM} ${OBJECTDIR}/_ext/1767308377/port.o.d 
	@${RM} ${OBJECTDIR}/_ext/1767308377/port.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1767308377/port.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1767308377/port.o.d" -o ${OBJECTDIR}/_ext/1767308377/port.o ../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX/port.c     
	
${OBJECTDIR}/_ext/1092921775/croutine.o: ../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/croutine.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1092921775" 
	@${RM} ${OBJECTDIR}/_ext/1092921775/croutine.o.d 
	@${RM} ${OBJECTDIR}/_ext/1092921775/croutine.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1092921775/croutine.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1092921775/croutine.o.d" -o ${OBJECTDIR}/_ext/1092921775/croutine.o ../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/croutine.c     
	
${OBJECTDIR}/_ext/1092921775/list.o: ../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/list.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1092921775" 
	@${RM} ${OBJECTDIR}/_ext/1092921775/list.o.d 
	@${RM} ${OBJECTDIR}/_ext/1092921775/list.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1092921775/list.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1092921775/list.o.d" -o ${OBJECTDIR}/_ext/1092921775/list.o ../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/list.c     
	
${OBJECTDIR}/_ext/1092921775/queue.o: ../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/queue.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1092921775" 
	@${RM} ${OBJECTDIR}/_ext/1092921775/queue.o.d 
	@${RM} ${OBJECTDIR}/_ext/1092921775/queue.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1092921775/queue.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1092921775/queue.o.d" -o ${OBJECTDIR}/_ext/1092921775/queue.o ../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/queue.c     
	
${OBJECTDIR}/_ext/1092921775/tasks.o: ../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1092921775" 
	@${RM} ${OBJECTDIR}/_ext/1092921775/tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1092921775/tasks.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1092921775/tasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1092921775/tasks.o.d" -o ${OBJECTDIR}/_ext/1092921775/tasks.o ../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/tasks.c     
	
${OBJECTDIR}/_ext/1092921775/timers.o: ../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/timers.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1092921775" 
	@${RM} ${OBJECTDIR}/_ext/1092921775/timers.o.d 
	@${RM} ${OBJECTDIR}/_ext/1092921775/timers.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1092921775/timers.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1092921775/timers.o.d" -o ${OBJECTDIR}/_ext/1092921775/timers.o ../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/timers.c     
	
${OBJECTDIR}/_ext/1092921775/event_groups.o: ../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/event_groups.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1092921775" 
	@${RM} ${OBJECTDIR}/_ext/1092921775/event_groups.o.d 
	@${RM} ${OBJECTDIR}/_ext/1092921775/event_groups.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1092921775/event_groups.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -D__XC -I"../src" -I"../src/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/framework" -I"../src/system_config/rover/framework" -I"../src/system_config/rover" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/portable/MPLAB/PIC32MX" -I"../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/include" -MMD -MF "${OBJECTDIR}/_ext/1092921775/event_groups.o.d" -o ${OBJECTDIR}/_ext/1092921775/event_groups.o ../../../../../../../../../microchip/harmony/v1_06_02/third_party/rtos/FreeRTOS/Source/event_groups.c     
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compileCPP
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/rover1.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk  ../../../../../../../../../microchip/harmony/v1_06_02/bin/framework/peripheral/PIC32MX795F512L_peripherals.a  
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mdebugger -D__MPLAB_DEBUGGER_PK3=1 -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/rover1.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}    ..\..\..\..\..\..\..\..\..\microchip\harmony\v1_06_02\bin\framework\peripheral\PIC32MX795F512L_peripherals.a          -mreserve=boot@0x1FC02000:0x1FC02FEF -mreserve=boot@0x1FC02000:0x1FC024FF  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_PK3=1,--defsym=_min_heap_size=0,--gc-sections,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/rover1.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk  ../../../../../../../../../microchip/harmony/v1_06_02/bin/framework/peripheral/PIC32MX795F512L_peripherals.a 
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/rover1.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}    ..\..\..\..\..\..\..\..\..\microchip\harmony\v1_06_02\bin\framework\peripheral\PIC32MX795F512L_peripherals.a        -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=_min_heap_size=0,--gc-sections,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml
	${MP_CC_DIR}\\xc32-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/rover1.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} 
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/rover
	${RM} -r dist/rover

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
