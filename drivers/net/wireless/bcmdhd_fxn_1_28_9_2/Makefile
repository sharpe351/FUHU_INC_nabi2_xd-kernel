DHDCFLAGS = -Wall -Wstrict-prototypes -Dlinux -DBCMDRIVER -DBCMDBG   \
	-DBCMDONGLEHOST -DUNRELEASEDCHIP -DBCMDMA32 -DBCMFILEIMAGE            \
	-DDHDTHREAD -DBDC -DBCMSDIO -DBCMLXSDMMC -DBCMPLATFORM_BUS   \
	-DWIFI_ACT_FRAME -DARP_OFFLOAD_SUPPORT -DDHD_BCMEVENTS -DSHOW_EVENTS    \
	-DENABLE_INSMOD_NO_FW_LOAD -DEMBEDDED_PLATFORM -DPNO_SUPPORT     \
	-DSET_RANDOM_MAC_SOFTAP -DKEEP_ALIVE -DPKT_FILTER_SUPPORT    \
    -DDHD_DEBUG -DSDTEST \
    -DCUSTOMER_HW2 \
	-Idrivers/net/wireless/bcmdhd_fxn_1_28_9_2 -Idrivers/net/wireless/bcmdhd_fxn_1_28_9_2/include

# USED SDIO ISR:
#DHDCFLAGS += -DSDIO_ISR_THREAD

# TO USE OOB: (NVRAM also need to be updated)
#-DOOB_INTR_ONLY -DHW_OOB -DMMC_SDIO_ABORT
ifeq ($(CONFIG_BCMDHD_HW_OOB),y)
DHDCFLAGS += -DOOB_INTR_ONLY -DHW_OOB -DMMC_SDIO_ABORT
else
DHDCFLAGS += -DSDIO_ISR_THREAD
endif

# NEEDED:
ifeq ($(CONFIG_BCMDHD_WIFI_CONTROL_FUNC),y)
DHDCFLAGS += -DCONFIG_WIFI_CONTROL_FUNC
endif

DHDCFLAGS += -DPROP_TXSTATUS
DHDCFLAGS += -DWL_CFG80211_SYNC_GON -DESCAN_RESULT_PATCH

#-DGET_CUSTOM_MAC_ENABLE 
#-DDHD_USE_IDLECOUNT  -DROAM_ENABLE -DVSDB 
#-DWL_CFG80211_VSDB_PRIORITIZE_SCAN_REQUEST -DAMPDU_HOSTREORDER
#-DHT40_GO
#-DTOE 

# OTHER FLAGS (CHECK):
DHDCFLAGS += -DLINUX -DWLLXIW

DHDOFILES = aiutils.o bcmsdh_sdmmc_linux.o dhd_linux.o siutils.o bcmutils.o   \
	dhd_linux_sched.o dhd_sdio.o bcmwifi_channels.o bcmevent.o hndpmu.o   \
	bcmsdh.o dhd_cdc.o bcmsdh_linux.o dhd_common.o linux_osl.o            \
	bcmsdh_sdmmc.o dhd_custom_gpio.o sbutils.o wldev_common.o wl_android.o

obj-$(CONFIG_BCMDHD_FXN_1_28_9_2) += bcmdhd.o

ifeq ($(KBUILD_SRC),)
DHD_SRC = $(src)
else
DHD_SRC = $(KBUILD_SRC)/$(src)
endif
ifneq ($(wildcard $(DHD_SRC)/dhd_nvram.obj),)
include $(src)/makefile.my
endif

bcmdhd-objs += $(DHDOFILES)
ifneq ($(CONFIG_WIRELESS_EXT),)
bcmdhd-objs += wl_iw.o
DHDCFLAGS += -DSOFTAP -DUSE_IW
#-DWL_WIRELESS_EXT 
endif

ifeq ($(CONFIG_CFG80211),y)
bcmdhd-objs += wl_cfg80211.o wl_cfgp2p.o wl_linux_mon.o dhd_cfg80211.o
DHDCFLAGS += -DWL_CFG80211 -DWL_CFG80211_STA_EVENT -DWL_ENABLE_P2P_IF -DWLP2P
#DHDCFLAGS += -DWL_CFG80211 -DWL_ENABLE_P2P_IF -DWLP2P
#DHDCFLAGS += -DCUSTOM_ROAM_TRIGGER_SETTING=-65
#DHDCFLAGS += -DCUSTOM_ROAM_DELTA_SETTING=15
endif

ifeq ($(CONFIG_DHD_USE_SCHED_SCAN),y)
DHDCFLAGS += -DWL_SCHED_SCAN
endif
EXTRA_CFLAGS = $(DHDCFLAGS)
#ifeq ($(CONFIG_BCMDHD_FXN_1_28_9_2),m)
EXTRA_LDFLAGS += --strip-debug
#endif
