vendor := $(srctree)/$(src)

ifneq "$(wildcard $(vendor)/qcom)" ""
	ifneq ($(CONFIG_OF_IGNORE_QCOM_DTS),y)
		subdir-y += qcom
	endif
endif

subdir-y += xiaomi-msm8937
