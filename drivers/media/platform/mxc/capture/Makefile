obj-$(CONFIG_VIDEO_MXC_CSI_CAMERA)	 += fsl_csi.o csi_v4l2_capture.o
obj-$(CONFIG_VIDEO_VBASE_CSI_CAMERA) += fsl_csi.o vbase_v4l2_capture.o

ifeq ($(CONFIG_VIDEO_MXC_IPU_CAMERA),y)
	obj-$(CONFIG_VIDEO_MXC_CAPTURE) += mxc_v4l2_capture.o
	obj-$(CONFIG_MXC_IPU_PRP_VF_SDC) += ipu_prp_vf_sdc.o ipu_prp_vf_sdc_bg.o
	obj-$(CONFIG_MXC_IPU_DEVICE_QUEUE_SDC) += ipu_fg_overlay_sdc.o ipu_bg_overlay_sdc.o
	obj-$(CONFIG_MXC_IPU_PRP_ENC) += ipu_prp_enc.o ipu_still.o
	obj-$(CONFIG_MXC_IPU_CSI_ENC) += ipu_csi_enc.o ipu_still.o
endif

ov5640_camera-objs := ov5640.o
obj-$(CONFIG_MXC_CAMERA_OV5640) += ov5640_camera.o

ov5642_camera-objs := ov5642.o
obj-$(CONFIG_MXC_CAMERA_OV5642) += ov5642_camera.o

vbase_camera-objs := vbase.o
obj-$(CONFIG_MXC_CAMERA_VBASE) += vbase_camera.o

ov5640_camera_mipi-objs := ov5640_mipi.o
obj-$(CONFIG_MXC_CAMERA_OV5640_MIPI) += ov5640_camera_mipi.o

tc358743_h2c_bridge-objs := tc358743_h2c.o
obj-$(CONFIG_MXC_HDMI_CSI2_TC358743) += tc358743_h2c_bridge.o

adv7180_tvin-objs := adv7180.o
obj-$(CONFIG_MXC_TVIN_ADV7180) += adv7180_tvin.o

mxc_vadc_tvin-objs := mxc_vadc.o
obj-$(CONFIG_MXC_VADC) += mxc_vadc_tvin.o

gs2971_video-objs := gs2971.o
obj-$(CONFIG_MXC_VIDEO_GS2971) += gs2971_video.o
