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
ifeq "$(wildcard nbproject/Makefile-local-PingPong_ECM144.mk)" "nbproject/Makefile-local-PingPong_ECM144.mk"
include nbproject/Makefile-local-PingPong_ECM144.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=PingPong_ECM144
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/PingPong_BSP.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/PingPong_BSP.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=../src/system_config/PingPong_ECM144/framework/driver/rtcc/src/drv_rtcc_static.c ../src/system_config/PingPong_ECM144/framework/driver/spi/dynamic/drv_spi_tasks.c ../src/system_config/PingPong_ECM144/framework/driver/spi/dynamic/drv_spi_master_rm_tasks.c ../src/system_config/PingPong_ECM144/framework/system/clk/src/sys_clk_static.c ../src/system_config/PingPong_ECM144/framework/system/ports/src/sys_ports_static.c ../src/system_config/PingPong_ECM144/system_init.c ../src/system_config/PingPong_ECM144/system_interrupt.c ../src/system_config/PingPong_ECM144/system_exceptions.c ../src/system_config/PingPong_ECM144/system_tasks.c ../src/app.c ../src/main.c ../src/PP_adc.c ../src/sqi_sst26vf032.c ../src/PP_1Wire.c ../../../../framework/driver/i2c/src/dynamic/drv_i2c.c ../../../../framework/driver/spi/src/dynamic/drv_spi.c ../../../../framework/driver/spi/src/dynamic/drv_spi_api.c ../../../../framework/driver/spi/src/drv_spi_sys_queue_fifo.c ../../../../framework/driver/tmr/src/dynamic/drv_tmr.c ../../../../framework/driver/usart/src/dynamic/drv_usart.c ../../../../framework/driver/usart/src/dynamic/drv_usart_buffer_queue.c ../../../../framework/driver/usart/src/dynamic/drv_usart_read_write.c ../../../../framework/driver/usb/usbhs/src/dynamic/drv_usbhs.c ../../../../framework/driver/usb/usbhs/src/dynamic/drv_usbhs_device.c ../../../../framework/system/devcon/src/sys_devcon.c ../../../../framework/system/devcon/src/sys_devcon_pic32mz.c ../../../../framework/system/devcon/src/sys_devcon_cache_pic32mz.S ../../../../framework/system/int/src/sys_int_pic32.c ../../../../framework/system/ports/src/sys_ports.c ../../../../framework/system/tmr/src/sys_tmr.c ../../../../framework/usb/src/dynamic/usb_device.c ../../../../framework/usb/src/dynamic/usb_device_cdc.c ../../../../framework/usb/src/dynamic/usb_device_cdc_acm.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/905106242/drv_rtcc_static.o ${OBJECTDIR}/_ext/343266613/drv_spi_tasks.o ${OBJECTDIR}/_ext/343266613/drv_spi_master_rm_tasks.o ${OBJECTDIR}/_ext/967239337/sys_clk_static.o ${OBJECTDIR}/_ext/998318151/sys_ports_static.o ${OBJECTDIR}/_ext/1106936416/system_init.o ${OBJECTDIR}/_ext/1106936416/system_interrupt.o ${OBJECTDIR}/_ext/1106936416/system_exceptions.o ${OBJECTDIR}/_ext/1106936416/system_tasks.o ${OBJECTDIR}/_ext/1360937237/app.o ${OBJECTDIR}/_ext/1360937237/main.o ${OBJECTDIR}/_ext/1360937237/PP_adc.o ${OBJECTDIR}/_ext/1360937237/sqi_sst26vf032.o ${OBJECTDIR}/_ext/1360937237/PP_1Wire.o ${OBJECTDIR}/_ext/280795049/drv_i2c.o ${OBJECTDIR}/_ext/568870469/drv_spi.o ${OBJECTDIR}/_ext/568870469/drv_spi_api.o ${OBJECTDIR}/_ext/465164171/drv_spi_sys_queue_fifo.o ${OBJECTDIR}/_ext/185269848/drv_tmr.o ${OBJECTDIR}/_ext/260586732/drv_usart.o ${OBJECTDIR}/_ext/260586732/drv_usart_buffer_queue.o ${OBJECTDIR}/_ext/260586732/drv_usart_read_write.o ${OBJECTDIR}/_ext/246898221/drv_usbhs.o ${OBJECTDIR}/_ext/246898221/drv_usbhs_device.o ${OBJECTDIR}/_ext/1271179505/sys_devcon.o ${OBJECTDIR}/_ext/1271179505/sys_devcon_pic32mz.o ${OBJECTDIR}/_ext/1271179505/sys_devcon_cache_pic32mz.o ${OBJECTDIR}/_ext/122796885/sys_int_pic32.o ${OBJECTDIR}/_ext/77319752/sys_ports.o ${OBJECTDIR}/_ext/1264926591/sys_tmr.o ${OBJECTDIR}/_ext/610166344/usb_device.o ${OBJECTDIR}/_ext/610166344/usb_device_cdc.o ${OBJECTDIR}/_ext/610166344/usb_device_cdc_acm.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/905106242/drv_rtcc_static.o.d ${OBJECTDIR}/_ext/343266613/drv_spi_tasks.o.d ${OBJECTDIR}/_ext/343266613/drv_spi_master_rm_tasks.o.d ${OBJECTDIR}/_ext/967239337/sys_clk_static.o.d ${OBJECTDIR}/_ext/998318151/sys_ports_static.o.d ${OBJECTDIR}/_ext/1106936416/system_init.o.d ${OBJECTDIR}/_ext/1106936416/system_interrupt.o.d ${OBJECTDIR}/_ext/1106936416/system_exceptions.o.d ${OBJECTDIR}/_ext/1106936416/system_tasks.o.d ${OBJECTDIR}/_ext/1360937237/app.o.d ${OBJECTDIR}/_ext/1360937237/main.o.d ${OBJECTDIR}/_ext/1360937237/PP_adc.o.d ${OBJECTDIR}/_ext/1360937237/sqi_sst26vf032.o.d ${OBJECTDIR}/_ext/1360937237/PP_1Wire.o.d ${OBJECTDIR}/_ext/280795049/drv_i2c.o.d ${OBJECTDIR}/_ext/568870469/drv_spi.o.d ${OBJECTDIR}/_ext/568870469/drv_spi_api.o.d ${OBJECTDIR}/_ext/465164171/drv_spi_sys_queue_fifo.o.d ${OBJECTDIR}/_ext/185269848/drv_tmr.o.d ${OBJECTDIR}/_ext/260586732/drv_usart.o.d ${OBJECTDIR}/_ext/260586732/drv_usart_buffer_queue.o.d ${OBJECTDIR}/_ext/260586732/drv_usart_read_write.o.d ${OBJECTDIR}/_ext/246898221/drv_usbhs.o.d ${OBJECTDIR}/_ext/246898221/drv_usbhs_device.o.d ${OBJECTDIR}/_ext/1271179505/sys_devcon.o.d ${OBJECTDIR}/_ext/1271179505/sys_devcon_pic32mz.o.d ${OBJECTDIR}/_ext/1271179505/sys_devcon_cache_pic32mz.o.d ${OBJECTDIR}/_ext/122796885/sys_int_pic32.o.d ${OBJECTDIR}/_ext/77319752/sys_ports.o.d ${OBJECTDIR}/_ext/1264926591/sys_tmr.o.d ${OBJECTDIR}/_ext/610166344/usb_device.o.d ${OBJECTDIR}/_ext/610166344/usb_device_cdc.o.d ${OBJECTDIR}/_ext/610166344/usb_device_cdc_acm.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/905106242/drv_rtcc_static.o ${OBJECTDIR}/_ext/343266613/drv_spi_tasks.o ${OBJECTDIR}/_ext/343266613/drv_spi_master_rm_tasks.o ${OBJECTDIR}/_ext/967239337/sys_clk_static.o ${OBJECTDIR}/_ext/998318151/sys_ports_static.o ${OBJECTDIR}/_ext/1106936416/system_init.o ${OBJECTDIR}/_ext/1106936416/system_interrupt.o ${OBJECTDIR}/_ext/1106936416/system_exceptions.o ${OBJECTDIR}/_ext/1106936416/system_tasks.o ${OBJECTDIR}/_ext/1360937237/app.o ${OBJECTDIR}/_ext/1360937237/main.o ${OBJECTDIR}/_ext/1360937237/PP_adc.o ${OBJECTDIR}/_ext/1360937237/sqi_sst26vf032.o ${OBJECTDIR}/_ext/1360937237/PP_1Wire.o ${OBJECTDIR}/_ext/280795049/drv_i2c.o ${OBJECTDIR}/_ext/568870469/drv_spi.o ${OBJECTDIR}/_ext/568870469/drv_spi_api.o ${OBJECTDIR}/_ext/465164171/drv_spi_sys_queue_fifo.o ${OBJECTDIR}/_ext/185269848/drv_tmr.o ${OBJECTDIR}/_ext/260586732/drv_usart.o ${OBJECTDIR}/_ext/260586732/drv_usart_buffer_queue.o ${OBJECTDIR}/_ext/260586732/drv_usart_read_write.o ${OBJECTDIR}/_ext/246898221/drv_usbhs.o ${OBJECTDIR}/_ext/246898221/drv_usbhs_device.o ${OBJECTDIR}/_ext/1271179505/sys_devcon.o ${OBJECTDIR}/_ext/1271179505/sys_devcon_pic32mz.o ${OBJECTDIR}/_ext/1271179505/sys_devcon_cache_pic32mz.o ${OBJECTDIR}/_ext/122796885/sys_int_pic32.o ${OBJECTDIR}/_ext/77319752/sys_ports.o ${OBJECTDIR}/_ext/1264926591/sys_tmr.o ${OBJECTDIR}/_ext/610166344/usb_device.o ${OBJECTDIR}/_ext/610166344/usb_device_cdc.o ${OBJECTDIR}/_ext/610166344/usb_device_cdc_acm.o

# Source Files
SOURCEFILES=../src/system_config/PingPong_ECM144/framework/driver/rtcc/src/drv_rtcc_static.c ../src/system_config/PingPong_ECM144/framework/driver/spi/dynamic/drv_spi_tasks.c ../src/system_config/PingPong_ECM144/framework/driver/spi/dynamic/drv_spi_master_rm_tasks.c ../src/system_config/PingPong_ECM144/framework/system/clk/src/sys_clk_static.c ../src/system_config/PingPong_ECM144/framework/system/ports/src/sys_ports_static.c ../src/system_config/PingPong_ECM144/system_init.c ../src/system_config/PingPong_ECM144/system_interrupt.c ../src/system_config/PingPong_ECM144/system_exceptions.c ../src/system_config/PingPong_ECM144/system_tasks.c ../src/app.c ../src/main.c ../src/PP_adc.c ../src/sqi_sst26vf032.c ../src/PP_1Wire.c ../../../../framework/driver/i2c/src/dynamic/drv_i2c.c ../../../../framework/driver/spi/src/dynamic/drv_spi.c ../../../../framework/driver/spi/src/dynamic/drv_spi_api.c ../../../../framework/driver/spi/src/drv_spi_sys_queue_fifo.c ../../../../framework/driver/tmr/src/dynamic/drv_tmr.c ../../../../framework/driver/usart/src/dynamic/drv_usart.c ../../../../framework/driver/usart/src/dynamic/drv_usart_buffer_queue.c ../../../../framework/driver/usart/src/dynamic/drv_usart_read_write.c ../../../../framework/driver/usb/usbhs/src/dynamic/drv_usbhs.c ../../../../framework/driver/usb/usbhs/src/dynamic/drv_usbhs_device.c ../../../../framework/system/devcon/src/sys_devcon.c ../../../../framework/system/devcon/src/sys_devcon_pic32mz.c ../../../../framework/system/devcon/src/sys_devcon_cache_pic32mz.S ../../../../framework/system/int/src/sys_int_pic32.c ../../../../framework/system/ports/src/sys_ports.c ../../../../framework/system/tmr/src/sys_tmr.c ../../../../framework/usb/src/dynamic/usb_device.c ../../../../framework/usb/src/dynamic/usb_device_cdc.c ../../../../framework/usb/src/dynamic/usb_device_cdc_acm.c


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
	${MAKE}  -f nbproject/Makefile-PingPong_ECM144.mk dist/${CND_CONF}/${IMAGE_TYPE}/PingPong_BSP.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=32MZ2048ECM144
MP_LINKER_FILE_OPTION=,--script="..\src\app_p32MZ.ld"
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assembleWithPreprocess
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/1271179505/sys_devcon_cache_pic32mz.o: ../../../../framework/system/devcon/src/sys_devcon_cache_pic32mz.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1271179505" 
	@${RM} ${OBJECTDIR}/_ext/1271179505/sys_devcon_cache_pic32mz.o.d 
	@${RM} ${OBJECTDIR}/_ext/1271179505/sys_devcon_cache_pic32mz.o 
	@${RM} ${OBJECTDIR}/_ext/1271179505/sys_devcon_cache_pic32mz.o.ok ${OBJECTDIR}/_ext/1271179505/sys_devcon_cache_pic32mz.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1271179505/sys_devcon_cache_pic32mz.o.d" "${OBJECTDIR}/_ext/1271179505/sys_devcon_cache_pic32mz.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC} $(MP_EXTRA_AS_PRE)  -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1271179505/sys_devcon_cache_pic32mz.o.d"  -o ${OBJECTDIR}/_ext/1271179505/sys_devcon_cache_pic32mz.o ../../../../framework/system/devcon/src/sys_devcon_cache_pic32mz.S    -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/1271179505/sys_devcon_cache_pic32mz.o.asm.d",--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--gdwarf-2,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_ICD3=1
	
else
${OBJECTDIR}/_ext/1271179505/sys_devcon_cache_pic32mz.o: ../../../../framework/system/devcon/src/sys_devcon_cache_pic32mz.S  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1271179505" 
	@${RM} ${OBJECTDIR}/_ext/1271179505/sys_devcon_cache_pic32mz.o.d 
	@${RM} ${OBJECTDIR}/_ext/1271179505/sys_devcon_cache_pic32mz.o 
	@${RM} ${OBJECTDIR}/_ext/1271179505/sys_devcon_cache_pic32mz.o.ok ${OBJECTDIR}/_ext/1271179505/sys_devcon_cache_pic32mz.o.err 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1271179505/sys_devcon_cache_pic32mz.o.d" "${OBJECTDIR}/_ext/1271179505/sys_devcon_cache_pic32mz.o.asm.d" -t $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC} $(MP_EXTRA_AS_PRE)  -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1271179505/sys_devcon_cache_pic32mz.o.d"  -o ${OBJECTDIR}/_ext/1271179505/sys_devcon_cache_pic32mz.o ../../../../framework/system/devcon/src/sys_devcon_cache_pic32mz.S    -Wa,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_AS_POST),-MD="${OBJECTDIR}/_ext/1271179505/sys_devcon_cache_pic32mz.o.asm.d",--gdwarf-2
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/905106242/drv_rtcc_static.o: ../src/system_config/PingPong_ECM144/framework/driver/rtcc/src/drv_rtcc_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/905106242" 
	@${RM} ${OBJECTDIR}/_ext/905106242/drv_rtcc_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/905106242/drv_rtcc_static.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/905106242/drv_rtcc_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/905106242/drv_rtcc_static.o.d" -o ${OBJECTDIR}/_ext/905106242/drv_rtcc_static.o ../src/system_config/PingPong_ECM144/framework/driver/rtcc/src/drv_rtcc_static.c     
	
${OBJECTDIR}/_ext/343266613/drv_spi_tasks.o: ../src/system_config/PingPong_ECM144/framework/driver/spi/dynamic/drv_spi_tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/343266613" 
	@${RM} ${OBJECTDIR}/_ext/343266613/drv_spi_tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/343266613/drv_spi_tasks.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/343266613/drv_spi_tasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/343266613/drv_spi_tasks.o.d" -o ${OBJECTDIR}/_ext/343266613/drv_spi_tasks.o ../src/system_config/PingPong_ECM144/framework/driver/spi/dynamic/drv_spi_tasks.c     
	
${OBJECTDIR}/_ext/343266613/drv_spi_master_rm_tasks.o: ../src/system_config/PingPong_ECM144/framework/driver/spi/dynamic/drv_spi_master_rm_tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/343266613" 
	@${RM} ${OBJECTDIR}/_ext/343266613/drv_spi_master_rm_tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/343266613/drv_spi_master_rm_tasks.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/343266613/drv_spi_master_rm_tasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/343266613/drv_spi_master_rm_tasks.o.d" -o ${OBJECTDIR}/_ext/343266613/drv_spi_master_rm_tasks.o ../src/system_config/PingPong_ECM144/framework/driver/spi/dynamic/drv_spi_master_rm_tasks.c     
	
${OBJECTDIR}/_ext/967239337/sys_clk_static.o: ../src/system_config/PingPong_ECM144/framework/system/clk/src/sys_clk_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/967239337" 
	@${RM} ${OBJECTDIR}/_ext/967239337/sys_clk_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/967239337/sys_clk_static.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/967239337/sys_clk_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/967239337/sys_clk_static.o.d" -o ${OBJECTDIR}/_ext/967239337/sys_clk_static.o ../src/system_config/PingPong_ECM144/framework/system/clk/src/sys_clk_static.c     
	
${OBJECTDIR}/_ext/998318151/sys_ports_static.o: ../src/system_config/PingPong_ECM144/framework/system/ports/src/sys_ports_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/998318151" 
	@${RM} ${OBJECTDIR}/_ext/998318151/sys_ports_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/998318151/sys_ports_static.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/998318151/sys_ports_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/998318151/sys_ports_static.o.d" -o ${OBJECTDIR}/_ext/998318151/sys_ports_static.o ../src/system_config/PingPong_ECM144/framework/system/ports/src/sys_ports_static.c     
	
${OBJECTDIR}/_ext/1106936416/system_init.o: ../src/system_config/PingPong_ECM144/system_init.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1106936416" 
	@${RM} ${OBJECTDIR}/_ext/1106936416/system_init.o.d 
	@${RM} ${OBJECTDIR}/_ext/1106936416/system_init.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1106936416/system_init.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/1106936416/system_init.o.d" -o ${OBJECTDIR}/_ext/1106936416/system_init.o ../src/system_config/PingPong_ECM144/system_init.c     
	
${OBJECTDIR}/_ext/1106936416/system_interrupt.o: ../src/system_config/PingPong_ECM144/system_interrupt.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1106936416" 
	@${RM} ${OBJECTDIR}/_ext/1106936416/system_interrupt.o.d 
	@${RM} ${OBJECTDIR}/_ext/1106936416/system_interrupt.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1106936416/system_interrupt.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/1106936416/system_interrupt.o.d" -o ${OBJECTDIR}/_ext/1106936416/system_interrupt.o ../src/system_config/PingPong_ECM144/system_interrupt.c     
	
${OBJECTDIR}/_ext/1106936416/system_exceptions.o: ../src/system_config/PingPong_ECM144/system_exceptions.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1106936416" 
	@${RM} ${OBJECTDIR}/_ext/1106936416/system_exceptions.o.d 
	@${RM} ${OBJECTDIR}/_ext/1106936416/system_exceptions.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1106936416/system_exceptions.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/1106936416/system_exceptions.o.d" -o ${OBJECTDIR}/_ext/1106936416/system_exceptions.o ../src/system_config/PingPong_ECM144/system_exceptions.c     
	
${OBJECTDIR}/_ext/1106936416/system_tasks.o: ../src/system_config/PingPong_ECM144/system_tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1106936416" 
	@${RM} ${OBJECTDIR}/_ext/1106936416/system_tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1106936416/system_tasks.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1106936416/system_tasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/1106936416/system_tasks.o.d" -o ${OBJECTDIR}/_ext/1106936416/system_tasks.o ../src/system_config/PingPong_ECM144/system_tasks.c     
	
${OBJECTDIR}/_ext/1360937237/app.o: ../src/app.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/app.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/app.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/app.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/1360937237/app.o.d" -o ${OBJECTDIR}/_ext/1360937237/app.o ../src/app.c     
	
${OBJECTDIR}/_ext/1360937237/main.o: ../src/main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/1360937237/main.o.d" -o ${OBJECTDIR}/_ext/1360937237/main.o ../src/main.c     
	
${OBJECTDIR}/_ext/1360937237/PP_adc.o: ../src/PP_adc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/PP_adc.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/PP_adc.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/PP_adc.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/1360937237/PP_adc.o.d" -o ${OBJECTDIR}/_ext/1360937237/PP_adc.o ../src/PP_adc.c     
	
${OBJECTDIR}/_ext/1360937237/sqi_sst26vf032.o: ../src/sqi_sst26vf032.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/sqi_sst26vf032.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/sqi_sst26vf032.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/sqi_sst26vf032.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/1360937237/sqi_sst26vf032.o.d" -o ${OBJECTDIR}/_ext/1360937237/sqi_sst26vf032.o ../src/sqi_sst26vf032.c     
	
${OBJECTDIR}/_ext/1360937237/PP_1Wire.o: ../src/PP_1Wire.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/PP_1Wire.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/PP_1Wire.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/PP_1Wire.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/1360937237/PP_1Wire.o.d" -o ${OBJECTDIR}/_ext/1360937237/PP_1Wire.o ../src/PP_1Wire.c     
	
${OBJECTDIR}/_ext/280795049/drv_i2c.o: ../../../../framework/driver/i2c/src/dynamic/drv_i2c.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/280795049" 
	@${RM} ${OBJECTDIR}/_ext/280795049/drv_i2c.o.d 
	@${RM} ${OBJECTDIR}/_ext/280795049/drv_i2c.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/280795049/drv_i2c.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/280795049/drv_i2c.o.d" -o ${OBJECTDIR}/_ext/280795049/drv_i2c.o ../../../../framework/driver/i2c/src/dynamic/drv_i2c.c     
	
${OBJECTDIR}/_ext/568870469/drv_spi.o: ../../../../framework/driver/spi/src/dynamic/drv_spi.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/568870469" 
	@${RM} ${OBJECTDIR}/_ext/568870469/drv_spi.o.d 
	@${RM} ${OBJECTDIR}/_ext/568870469/drv_spi.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/568870469/drv_spi.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/568870469/drv_spi.o.d" -o ${OBJECTDIR}/_ext/568870469/drv_spi.o ../../../../framework/driver/spi/src/dynamic/drv_spi.c     
	
${OBJECTDIR}/_ext/568870469/drv_spi_api.o: ../../../../framework/driver/spi/src/dynamic/drv_spi_api.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/568870469" 
	@${RM} ${OBJECTDIR}/_ext/568870469/drv_spi_api.o.d 
	@${RM} ${OBJECTDIR}/_ext/568870469/drv_spi_api.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/568870469/drv_spi_api.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/568870469/drv_spi_api.o.d" -o ${OBJECTDIR}/_ext/568870469/drv_spi_api.o ../../../../framework/driver/spi/src/dynamic/drv_spi_api.c     
	
${OBJECTDIR}/_ext/465164171/drv_spi_sys_queue_fifo.o: ../../../../framework/driver/spi/src/drv_spi_sys_queue_fifo.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/465164171" 
	@${RM} ${OBJECTDIR}/_ext/465164171/drv_spi_sys_queue_fifo.o.d 
	@${RM} ${OBJECTDIR}/_ext/465164171/drv_spi_sys_queue_fifo.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/465164171/drv_spi_sys_queue_fifo.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/465164171/drv_spi_sys_queue_fifo.o.d" -o ${OBJECTDIR}/_ext/465164171/drv_spi_sys_queue_fifo.o ../../../../framework/driver/spi/src/drv_spi_sys_queue_fifo.c     
	
${OBJECTDIR}/_ext/185269848/drv_tmr.o: ../../../../framework/driver/tmr/src/dynamic/drv_tmr.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/185269848" 
	@${RM} ${OBJECTDIR}/_ext/185269848/drv_tmr.o.d 
	@${RM} ${OBJECTDIR}/_ext/185269848/drv_tmr.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/185269848/drv_tmr.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/185269848/drv_tmr.o.d" -o ${OBJECTDIR}/_ext/185269848/drv_tmr.o ../../../../framework/driver/tmr/src/dynamic/drv_tmr.c     
	
${OBJECTDIR}/_ext/260586732/drv_usart.o: ../../../../framework/driver/usart/src/dynamic/drv_usart.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/260586732" 
	@${RM} ${OBJECTDIR}/_ext/260586732/drv_usart.o.d 
	@${RM} ${OBJECTDIR}/_ext/260586732/drv_usart.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/260586732/drv_usart.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/260586732/drv_usart.o.d" -o ${OBJECTDIR}/_ext/260586732/drv_usart.o ../../../../framework/driver/usart/src/dynamic/drv_usart.c     
	
${OBJECTDIR}/_ext/260586732/drv_usart_buffer_queue.o: ../../../../framework/driver/usart/src/dynamic/drv_usart_buffer_queue.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/260586732" 
	@${RM} ${OBJECTDIR}/_ext/260586732/drv_usart_buffer_queue.o.d 
	@${RM} ${OBJECTDIR}/_ext/260586732/drv_usart_buffer_queue.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/260586732/drv_usart_buffer_queue.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/260586732/drv_usart_buffer_queue.o.d" -o ${OBJECTDIR}/_ext/260586732/drv_usart_buffer_queue.o ../../../../framework/driver/usart/src/dynamic/drv_usart_buffer_queue.c     
	
${OBJECTDIR}/_ext/260586732/drv_usart_read_write.o: ../../../../framework/driver/usart/src/dynamic/drv_usart_read_write.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/260586732" 
	@${RM} ${OBJECTDIR}/_ext/260586732/drv_usart_read_write.o.d 
	@${RM} ${OBJECTDIR}/_ext/260586732/drv_usart_read_write.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/260586732/drv_usart_read_write.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/260586732/drv_usart_read_write.o.d" -o ${OBJECTDIR}/_ext/260586732/drv_usart_read_write.o ../../../../framework/driver/usart/src/dynamic/drv_usart_read_write.c     
	
${OBJECTDIR}/_ext/246898221/drv_usbhs.o: ../../../../framework/driver/usb/usbhs/src/dynamic/drv_usbhs.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/246898221" 
	@${RM} ${OBJECTDIR}/_ext/246898221/drv_usbhs.o.d 
	@${RM} ${OBJECTDIR}/_ext/246898221/drv_usbhs.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/246898221/drv_usbhs.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/246898221/drv_usbhs.o.d" -o ${OBJECTDIR}/_ext/246898221/drv_usbhs.o ../../../../framework/driver/usb/usbhs/src/dynamic/drv_usbhs.c     
	
${OBJECTDIR}/_ext/246898221/drv_usbhs_device.o: ../../../../framework/driver/usb/usbhs/src/dynamic/drv_usbhs_device.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/246898221" 
	@${RM} ${OBJECTDIR}/_ext/246898221/drv_usbhs_device.o.d 
	@${RM} ${OBJECTDIR}/_ext/246898221/drv_usbhs_device.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/246898221/drv_usbhs_device.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/246898221/drv_usbhs_device.o.d" -o ${OBJECTDIR}/_ext/246898221/drv_usbhs_device.o ../../../../framework/driver/usb/usbhs/src/dynamic/drv_usbhs_device.c     
	
${OBJECTDIR}/_ext/1271179505/sys_devcon.o: ../../../../framework/system/devcon/src/sys_devcon.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1271179505" 
	@${RM} ${OBJECTDIR}/_ext/1271179505/sys_devcon.o.d 
	@${RM} ${OBJECTDIR}/_ext/1271179505/sys_devcon.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1271179505/sys_devcon.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/1271179505/sys_devcon.o.d" -o ${OBJECTDIR}/_ext/1271179505/sys_devcon.o ../../../../framework/system/devcon/src/sys_devcon.c     
	
${OBJECTDIR}/_ext/1271179505/sys_devcon_pic32mz.o: ../../../../framework/system/devcon/src/sys_devcon_pic32mz.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1271179505" 
	@${RM} ${OBJECTDIR}/_ext/1271179505/sys_devcon_pic32mz.o.d 
	@${RM} ${OBJECTDIR}/_ext/1271179505/sys_devcon_pic32mz.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1271179505/sys_devcon_pic32mz.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/1271179505/sys_devcon_pic32mz.o.d" -o ${OBJECTDIR}/_ext/1271179505/sys_devcon_pic32mz.o ../../../../framework/system/devcon/src/sys_devcon_pic32mz.c     
	
${OBJECTDIR}/_ext/122796885/sys_int_pic32.o: ../../../../framework/system/int/src/sys_int_pic32.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/122796885" 
	@${RM} ${OBJECTDIR}/_ext/122796885/sys_int_pic32.o.d 
	@${RM} ${OBJECTDIR}/_ext/122796885/sys_int_pic32.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/122796885/sys_int_pic32.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/122796885/sys_int_pic32.o.d" -o ${OBJECTDIR}/_ext/122796885/sys_int_pic32.o ../../../../framework/system/int/src/sys_int_pic32.c     
	
${OBJECTDIR}/_ext/77319752/sys_ports.o: ../../../../framework/system/ports/src/sys_ports.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/77319752" 
	@${RM} ${OBJECTDIR}/_ext/77319752/sys_ports.o.d 
	@${RM} ${OBJECTDIR}/_ext/77319752/sys_ports.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/77319752/sys_ports.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/77319752/sys_ports.o.d" -o ${OBJECTDIR}/_ext/77319752/sys_ports.o ../../../../framework/system/ports/src/sys_ports.c     
	
${OBJECTDIR}/_ext/1264926591/sys_tmr.o: ../../../../framework/system/tmr/src/sys_tmr.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1264926591" 
	@${RM} ${OBJECTDIR}/_ext/1264926591/sys_tmr.o.d 
	@${RM} ${OBJECTDIR}/_ext/1264926591/sys_tmr.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1264926591/sys_tmr.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/1264926591/sys_tmr.o.d" -o ${OBJECTDIR}/_ext/1264926591/sys_tmr.o ../../../../framework/system/tmr/src/sys_tmr.c     
	
${OBJECTDIR}/_ext/610166344/usb_device.o: ../../../../framework/usb/src/dynamic/usb_device.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/610166344" 
	@${RM} ${OBJECTDIR}/_ext/610166344/usb_device.o.d 
	@${RM} ${OBJECTDIR}/_ext/610166344/usb_device.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/610166344/usb_device.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/610166344/usb_device.o.d" -o ${OBJECTDIR}/_ext/610166344/usb_device.o ../../../../framework/usb/src/dynamic/usb_device.c     
	
${OBJECTDIR}/_ext/610166344/usb_device_cdc.o: ../../../../framework/usb/src/dynamic/usb_device_cdc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/610166344" 
	@${RM} ${OBJECTDIR}/_ext/610166344/usb_device_cdc.o.d 
	@${RM} ${OBJECTDIR}/_ext/610166344/usb_device_cdc.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/610166344/usb_device_cdc.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/610166344/usb_device_cdc.o.d" -o ${OBJECTDIR}/_ext/610166344/usb_device_cdc.o ../../../../framework/usb/src/dynamic/usb_device_cdc.c     
	
${OBJECTDIR}/_ext/610166344/usb_device_cdc_acm.o: ../../../../framework/usb/src/dynamic/usb_device_cdc_acm.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/610166344" 
	@${RM} ${OBJECTDIR}/_ext/610166344/usb_device_cdc_acm.o.d 
	@${RM} ${OBJECTDIR}/_ext/610166344/usb_device_cdc_acm.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/610166344/usb_device_cdc_acm.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/610166344/usb_device_cdc_acm.o.d" -o ${OBJECTDIR}/_ext/610166344/usb_device_cdc_acm.o ../../../../framework/usb/src/dynamic/usb_device_cdc_acm.c     
	
else
${OBJECTDIR}/_ext/905106242/drv_rtcc_static.o: ../src/system_config/PingPong_ECM144/framework/driver/rtcc/src/drv_rtcc_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/905106242" 
	@${RM} ${OBJECTDIR}/_ext/905106242/drv_rtcc_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/905106242/drv_rtcc_static.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/905106242/drv_rtcc_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/905106242/drv_rtcc_static.o.d" -o ${OBJECTDIR}/_ext/905106242/drv_rtcc_static.o ../src/system_config/PingPong_ECM144/framework/driver/rtcc/src/drv_rtcc_static.c     
	
${OBJECTDIR}/_ext/343266613/drv_spi_tasks.o: ../src/system_config/PingPong_ECM144/framework/driver/spi/dynamic/drv_spi_tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/343266613" 
	@${RM} ${OBJECTDIR}/_ext/343266613/drv_spi_tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/343266613/drv_spi_tasks.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/343266613/drv_spi_tasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/343266613/drv_spi_tasks.o.d" -o ${OBJECTDIR}/_ext/343266613/drv_spi_tasks.o ../src/system_config/PingPong_ECM144/framework/driver/spi/dynamic/drv_spi_tasks.c     
	
${OBJECTDIR}/_ext/343266613/drv_spi_master_rm_tasks.o: ../src/system_config/PingPong_ECM144/framework/driver/spi/dynamic/drv_spi_master_rm_tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/343266613" 
	@${RM} ${OBJECTDIR}/_ext/343266613/drv_spi_master_rm_tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/343266613/drv_spi_master_rm_tasks.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/343266613/drv_spi_master_rm_tasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/343266613/drv_spi_master_rm_tasks.o.d" -o ${OBJECTDIR}/_ext/343266613/drv_spi_master_rm_tasks.o ../src/system_config/PingPong_ECM144/framework/driver/spi/dynamic/drv_spi_master_rm_tasks.c     
	
${OBJECTDIR}/_ext/967239337/sys_clk_static.o: ../src/system_config/PingPong_ECM144/framework/system/clk/src/sys_clk_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/967239337" 
	@${RM} ${OBJECTDIR}/_ext/967239337/sys_clk_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/967239337/sys_clk_static.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/967239337/sys_clk_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/967239337/sys_clk_static.o.d" -o ${OBJECTDIR}/_ext/967239337/sys_clk_static.o ../src/system_config/PingPong_ECM144/framework/system/clk/src/sys_clk_static.c     
	
${OBJECTDIR}/_ext/998318151/sys_ports_static.o: ../src/system_config/PingPong_ECM144/framework/system/ports/src/sys_ports_static.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/998318151" 
	@${RM} ${OBJECTDIR}/_ext/998318151/sys_ports_static.o.d 
	@${RM} ${OBJECTDIR}/_ext/998318151/sys_ports_static.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/998318151/sys_ports_static.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/998318151/sys_ports_static.o.d" -o ${OBJECTDIR}/_ext/998318151/sys_ports_static.o ../src/system_config/PingPong_ECM144/framework/system/ports/src/sys_ports_static.c     
	
${OBJECTDIR}/_ext/1106936416/system_init.o: ../src/system_config/PingPong_ECM144/system_init.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1106936416" 
	@${RM} ${OBJECTDIR}/_ext/1106936416/system_init.o.d 
	@${RM} ${OBJECTDIR}/_ext/1106936416/system_init.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1106936416/system_init.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/1106936416/system_init.o.d" -o ${OBJECTDIR}/_ext/1106936416/system_init.o ../src/system_config/PingPong_ECM144/system_init.c     
	
${OBJECTDIR}/_ext/1106936416/system_interrupt.o: ../src/system_config/PingPong_ECM144/system_interrupt.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1106936416" 
	@${RM} ${OBJECTDIR}/_ext/1106936416/system_interrupt.o.d 
	@${RM} ${OBJECTDIR}/_ext/1106936416/system_interrupt.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1106936416/system_interrupt.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/1106936416/system_interrupt.o.d" -o ${OBJECTDIR}/_ext/1106936416/system_interrupt.o ../src/system_config/PingPong_ECM144/system_interrupt.c     
	
${OBJECTDIR}/_ext/1106936416/system_exceptions.o: ../src/system_config/PingPong_ECM144/system_exceptions.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1106936416" 
	@${RM} ${OBJECTDIR}/_ext/1106936416/system_exceptions.o.d 
	@${RM} ${OBJECTDIR}/_ext/1106936416/system_exceptions.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1106936416/system_exceptions.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/1106936416/system_exceptions.o.d" -o ${OBJECTDIR}/_ext/1106936416/system_exceptions.o ../src/system_config/PingPong_ECM144/system_exceptions.c     
	
${OBJECTDIR}/_ext/1106936416/system_tasks.o: ../src/system_config/PingPong_ECM144/system_tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1106936416" 
	@${RM} ${OBJECTDIR}/_ext/1106936416/system_tasks.o.d 
	@${RM} ${OBJECTDIR}/_ext/1106936416/system_tasks.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1106936416/system_tasks.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/1106936416/system_tasks.o.d" -o ${OBJECTDIR}/_ext/1106936416/system_tasks.o ../src/system_config/PingPong_ECM144/system_tasks.c     
	
${OBJECTDIR}/_ext/1360937237/app.o: ../src/app.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/app.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/app.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/app.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/1360937237/app.o.d" -o ${OBJECTDIR}/_ext/1360937237/app.o ../src/app.c     
	
${OBJECTDIR}/_ext/1360937237/main.o: ../src/main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/1360937237/main.o.d" -o ${OBJECTDIR}/_ext/1360937237/main.o ../src/main.c     
	
${OBJECTDIR}/_ext/1360937237/PP_adc.o: ../src/PP_adc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/PP_adc.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/PP_adc.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/PP_adc.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/1360937237/PP_adc.o.d" -o ${OBJECTDIR}/_ext/1360937237/PP_adc.o ../src/PP_adc.c     
	
${OBJECTDIR}/_ext/1360937237/sqi_sst26vf032.o: ../src/sqi_sst26vf032.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/sqi_sst26vf032.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/sqi_sst26vf032.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/sqi_sst26vf032.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/1360937237/sqi_sst26vf032.o.d" -o ${OBJECTDIR}/_ext/1360937237/sqi_sst26vf032.o ../src/sqi_sst26vf032.c     
	
${OBJECTDIR}/_ext/1360937237/PP_1Wire.o: ../src/PP_1Wire.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1360937237" 
	@${RM} ${OBJECTDIR}/_ext/1360937237/PP_1Wire.o.d 
	@${RM} ${OBJECTDIR}/_ext/1360937237/PP_1Wire.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/PP_1Wire.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/1360937237/PP_1Wire.o.d" -o ${OBJECTDIR}/_ext/1360937237/PP_1Wire.o ../src/PP_1Wire.c     
	
${OBJECTDIR}/_ext/280795049/drv_i2c.o: ../../../../framework/driver/i2c/src/dynamic/drv_i2c.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/280795049" 
	@${RM} ${OBJECTDIR}/_ext/280795049/drv_i2c.o.d 
	@${RM} ${OBJECTDIR}/_ext/280795049/drv_i2c.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/280795049/drv_i2c.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/280795049/drv_i2c.o.d" -o ${OBJECTDIR}/_ext/280795049/drv_i2c.o ../../../../framework/driver/i2c/src/dynamic/drv_i2c.c     
	
${OBJECTDIR}/_ext/568870469/drv_spi.o: ../../../../framework/driver/spi/src/dynamic/drv_spi.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/568870469" 
	@${RM} ${OBJECTDIR}/_ext/568870469/drv_spi.o.d 
	@${RM} ${OBJECTDIR}/_ext/568870469/drv_spi.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/568870469/drv_spi.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/568870469/drv_spi.o.d" -o ${OBJECTDIR}/_ext/568870469/drv_spi.o ../../../../framework/driver/spi/src/dynamic/drv_spi.c     
	
${OBJECTDIR}/_ext/568870469/drv_spi_api.o: ../../../../framework/driver/spi/src/dynamic/drv_spi_api.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/568870469" 
	@${RM} ${OBJECTDIR}/_ext/568870469/drv_spi_api.o.d 
	@${RM} ${OBJECTDIR}/_ext/568870469/drv_spi_api.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/568870469/drv_spi_api.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/568870469/drv_spi_api.o.d" -o ${OBJECTDIR}/_ext/568870469/drv_spi_api.o ../../../../framework/driver/spi/src/dynamic/drv_spi_api.c     
	
${OBJECTDIR}/_ext/465164171/drv_spi_sys_queue_fifo.o: ../../../../framework/driver/spi/src/drv_spi_sys_queue_fifo.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/465164171" 
	@${RM} ${OBJECTDIR}/_ext/465164171/drv_spi_sys_queue_fifo.o.d 
	@${RM} ${OBJECTDIR}/_ext/465164171/drv_spi_sys_queue_fifo.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/465164171/drv_spi_sys_queue_fifo.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/465164171/drv_spi_sys_queue_fifo.o.d" -o ${OBJECTDIR}/_ext/465164171/drv_spi_sys_queue_fifo.o ../../../../framework/driver/spi/src/drv_spi_sys_queue_fifo.c     
	
${OBJECTDIR}/_ext/185269848/drv_tmr.o: ../../../../framework/driver/tmr/src/dynamic/drv_tmr.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/185269848" 
	@${RM} ${OBJECTDIR}/_ext/185269848/drv_tmr.o.d 
	@${RM} ${OBJECTDIR}/_ext/185269848/drv_tmr.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/185269848/drv_tmr.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/185269848/drv_tmr.o.d" -o ${OBJECTDIR}/_ext/185269848/drv_tmr.o ../../../../framework/driver/tmr/src/dynamic/drv_tmr.c     
	
${OBJECTDIR}/_ext/260586732/drv_usart.o: ../../../../framework/driver/usart/src/dynamic/drv_usart.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/260586732" 
	@${RM} ${OBJECTDIR}/_ext/260586732/drv_usart.o.d 
	@${RM} ${OBJECTDIR}/_ext/260586732/drv_usart.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/260586732/drv_usart.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/260586732/drv_usart.o.d" -o ${OBJECTDIR}/_ext/260586732/drv_usart.o ../../../../framework/driver/usart/src/dynamic/drv_usart.c     
	
${OBJECTDIR}/_ext/260586732/drv_usart_buffer_queue.o: ../../../../framework/driver/usart/src/dynamic/drv_usart_buffer_queue.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/260586732" 
	@${RM} ${OBJECTDIR}/_ext/260586732/drv_usart_buffer_queue.o.d 
	@${RM} ${OBJECTDIR}/_ext/260586732/drv_usart_buffer_queue.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/260586732/drv_usart_buffer_queue.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/260586732/drv_usart_buffer_queue.o.d" -o ${OBJECTDIR}/_ext/260586732/drv_usart_buffer_queue.o ../../../../framework/driver/usart/src/dynamic/drv_usart_buffer_queue.c     
	
${OBJECTDIR}/_ext/260586732/drv_usart_read_write.o: ../../../../framework/driver/usart/src/dynamic/drv_usart_read_write.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/260586732" 
	@${RM} ${OBJECTDIR}/_ext/260586732/drv_usart_read_write.o.d 
	@${RM} ${OBJECTDIR}/_ext/260586732/drv_usart_read_write.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/260586732/drv_usart_read_write.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/260586732/drv_usart_read_write.o.d" -o ${OBJECTDIR}/_ext/260586732/drv_usart_read_write.o ../../../../framework/driver/usart/src/dynamic/drv_usart_read_write.c     
	
${OBJECTDIR}/_ext/246898221/drv_usbhs.o: ../../../../framework/driver/usb/usbhs/src/dynamic/drv_usbhs.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/246898221" 
	@${RM} ${OBJECTDIR}/_ext/246898221/drv_usbhs.o.d 
	@${RM} ${OBJECTDIR}/_ext/246898221/drv_usbhs.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/246898221/drv_usbhs.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/246898221/drv_usbhs.o.d" -o ${OBJECTDIR}/_ext/246898221/drv_usbhs.o ../../../../framework/driver/usb/usbhs/src/dynamic/drv_usbhs.c     
	
${OBJECTDIR}/_ext/246898221/drv_usbhs_device.o: ../../../../framework/driver/usb/usbhs/src/dynamic/drv_usbhs_device.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/246898221" 
	@${RM} ${OBJECTDIR}/_ext/246898221/drv_usbhs_device.o.d 
	@${RM} ${OBJECTDIR}/_ext/246898221/drv_usbhs_device.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/246898221/drv_usbhs_device.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/246898221/drv_usbhs_device.o.d" -o ${OBJECTDIR}/_ext/246898221/drv_usbhs_device.o ../../../../framework/driver/usb/usbhs/src/dynamic/drv_usbhs_device.c     
	
${OBJECTDIR}/_ext/1271179505/sys_devcon.o: ../../../../framework/system/devcon/src/sys_devcon.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1271179505" 
	@${RM} ${OBJECTDIR}/_ext/1271179505/sys_devcon.o.d 
	@${RM} ${OBJECTDIR}/_ext/1271179505/sys_devcon.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1271179505/sys_devcon.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/1271179505/sys_devcon.o.d" -o ${OBJECTDIR}/_ext/1271179505/sys_devcon.o ../../../../framework/system/devcon/src/sys_devcon.c     
	
${OBJECTDIR}/_ext/1271179505/sys_devcon_pic32mz.o: ../../../../framework/system/devcon/src/sys_devcon_pic32mz.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1271179505" 
	@${RM} ${OBJECTDIR}/_ext/1271179505/sys_devcon_pic32mz.o.d 
	@${RM} ${OBJECTDIR}/_ext/1271179505/sys_devcon_pic32mz.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1271179505/sys_devcon_pic32mz.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/1271179505/sys_devcon_pic32mz.o.d" -o ${OBJECTDIR}/_ext/1271179505/sys_devcon_pic32mz.o ../../../../framework/system/devcon/src/sys_devcon_pic32mz.c     
	
${OBJECTDIR}/_ext/122796885/sys_int_pic32.o: ../../../../framework/system/int/src/sys_int_pic32.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/122796885" 
	@${RM} ${OBJECTDIR}/_ext/122796885/sys_int_pic32.o.d 
	@${RM} ${OBJECTDIR}/_ext/122796885/sys_int_pic32.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/122796885/sys_int_pic32.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/122796885/sys_int_pic32.o.d" -o ${OBJECTDIR}/_ext/122796885/sys_int_pic32.o ../../../../framework/system/int/src/sys_int_pic32.c     
	
${OBJECTDIR}/_ext/77319752/sys_ports.o: ../../../../framework/system/ports/src/sys_ports.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/77319752" 
	@${RM} ${OBJECTDIR}/_ext/77319752/sys_ports.o.d 
	@${RM} ${OBJECTDIR}/_ext/77319752/sys_ports.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/77319752/sys_ports.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/77319752/sys_ports.o.d" -o ${OBJECTDIR}/_ext/77319752/sys_ports.o ../../../../framework/system/ports/src/sys_ports.c     
	
${OBJECTDIR}/_ext/1264926591/sys_tmr.o: ../../../../framework/system/tmr/src/sys_tmr.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/1264926591" 
	@${RM} ${OBJECTDIR}/_ext/1264926591/sys_tmr.o.d 
	@${RM} ${OBJECTDIR}/_ext/1264926591/sys_tmr.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1264926591/sys_tmr.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/1264926591/sys_tmr.o.d" -o ${OBJECTDIR}/_ext/1264926591/sys_tmr.o ../../../../framework/system/tmr/src/sys_tmr.c     
	
${OBJECTDIR}/_ext/610166344/usb_device.o: ../../../../framework/usb/src/dynamic/usb_device.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/610166344" 
	@${RM} ${OBJECTDIR}/_ext/610166344/usb_device.o.d 
	@${RM} ${OBJECTDIR}/_ext/610166344/usb_device.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/610166344/usb_device.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/610166344/usb_device.o.d" -o ${OBJECTDIR}/_ext/610166344/usb_device.o ../../../../framework/usb/src/dynamic/usb_device.c     
	
${OBJECTDIR}/_ext/610166344/usb_device_cdc.o: ../../../../framework/usb/src/dynamic/usb_device_cdc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/610166344" 
	@${RM} ${OBJECTDIR}/_ext/610166344/usb_device_cdc.o.d 
	@${RM} ${OBJECTDIR}/_ext/610166344/usb_device_cdc.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/610166344/usb_device_cdc.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/610166344/usb_device_cdc.o.d" -o ${OBJECTDIR}/_ext/610166344/usb_device_cdc.o ../../../../framework/usb/src/dynamic/usb_device_cdc.c     
	
${OBJECTDIR}/_ext/610166344/usb_device_cdc_acm.o: ../../../../framework/usb/src/dynamic/usb_device_cdc_acm.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/_ext/610166344" 
	@${RM} ${OBJECTDIR}/_ext/610166344/usb_device_cdc_acm.o.d 
	@${RM} ${OBJECTDIR}/_ext/610166344/usb_device_cdc_acm.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/610166344/usb_device_cdc_acm.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -ffunction-sections -O1 -I"../src" -I"../src/system_config/PingPong_ECM144" -I"../../../../framework" -I"../src/system_config/PingPong_ECM144/framework" -MMD -MF "${OBJECTDIR}/_ext/610166344/usb_device_cdc_acm.o.d" -o ${OBJECTDIR}/_ext/610166344/usb_device_cdc_acm.o ../../../../framework/usb/src/dynamic/usb_device_cdc_acm.c     
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compileCPP
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/PingPong_BSP.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk  ../../../../bin/framework/peripheral/PIC32MZ2048ECM144_peripherals.a  ../src/app_p32MZ.ld
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mdebugger -D__MPLAB_DEBUGGER_ICD3=1 -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/PingPong_BSP.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}    ..\..\..\..\bin\framework\peripheral\PIC32MZ2048ECM144_peripherals.a            -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_ICD3=1,--defsym=_min_heap_size=0,--gc-sections,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/PingPong_BSP.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk  ../../../../bin/framework/peripheral/PIC32MZ2048ECM144_peripherals.a ../src/app_p32MZ.ld
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/PingPong_BSP.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}    ..\..\..\..\bin\framework\peripheral\PIC32MZ2048ECM144_peripherals.a        -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=_min_heap_size=0,--gc-sections,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml
	${MP_CC_DIR}\\xc32-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/PingPong_BSP.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} 
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/PingPong_ECM144
	${RM} -r dist/PingPong_ECM144

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
