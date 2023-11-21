all : flash

TARGET:= esp_mess_addon
# TARGET_EXT:=cpp

# ADDITIONAL_C_FILES+=TestClass.cpp

include ../../ch32v003fun/ch32v003fun.mk

# CFLAGS:= \
# 	-g -flto -ffunction-sections \
# 	-static-libgcc \
# 	-march=rv32ec \
# 	-mabi=ilp32e \
# 	-I/usr/include/newlib \
# 	-I$(CH32V003FUN) \
# 	-nostdlib \
# 	-I. -Wall

# CFLAGS+=-fno-rtti -DCPLUSPLUS

flash : cv_flash
clean : cv_clean
