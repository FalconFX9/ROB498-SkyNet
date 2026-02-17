#include <vpi/VPI.h>
#include <vpi/algo/StereoDisparity.h>
#include <vpi/algo/Rescale.h>
#include <vpi/algo/ConvertImageFormat.h>
#include <cstring>
#include <cstdint>
#include <cstdio>
#include <vector>

#define CHECK_STATUS(op) \
    do { \
        VPIStatus status = (op); \
        if (status != VPI_SUCCESS) { \
            char buffer[256]; \
            vpiGetLastStatusMessage(buffer, sizeof(buffer)); \
            fprintf(stderr, "[VPI ERROR] %s: %s\n", #op, buffer); \
        } \
    } while (0)

extern "C" {

    struct PipelineContext {
        VPIStream stream;
        VPIPayload stereoPayload;
        VPIImage leftFull, rightFull;
        VPIImage leftSmall, rightSmall;
        VPIImage dispSmall;
        int full_w, full_h, small_w, small_h;
        std::vector<uint8_t> dummyBuffer; 
    };

    PipelineContext* init_pipeline(int width, int height, int max_disp_input) {
        PipelineContext* ctx = new PipelineContext;
        ctx->full_w = width;
        ctx->full_h = height;
        ctx->small_w = width / 2;
        ctx->small_h = height / 2;
        int small_max_disp = max_disp_input / 2;

        ctx->dummyBuffer.resize(width * height, 0);

        CHECK_STATUS(vpiStreamCreate(0, &ctx->stream));

        VPIImageData params;
        memset(&params, 0, sizeof(params));
        params.format = VPI_IMAGE_FORMAT_U8;
        params.numPlanes = 1;
        params.planes[0].width = width;
        params.planes[0].height = height;
        params.planes[0].pitchBytes = width;
        params.planes[0].data = ctx->dummyBuffer.data(); 

        CHECK_STATUS(vpiImageCreateHostMemWrapper(&params, 0, &ctx->leftFull));
        CHECK_STATUS(vpiImageCreateHostMemWrapper(&params, 0, &ctx->rightFull));

        CHECK_STATUS(vpiImageCreate(ctx->small_w, ctx->small_h, VPI_IMAGE_FORMAT_U8, 0, &ctx->leftSmall));
        CHECK_STATUS(vpiImageCreate(ctx->small_w, ctx->small_h, VPI_IMAGE_FORMAT_U8, 0, &ctx->rightSmall));
        CHECK_STATUS(vpiImageCreate(ctx->small_w, ctx->small_h, VPI_IMAGE_FORMAT_U16, 0, &ctx->dispSmall));

        VPIStereoDisparityEstimatorCreationParams createParams;
        vpiInitStereoDisparityEstimatorCreationParams(&createParams); 
        createParams.maxDisparity = small_max_disp;
        CHECK_STATUS(vpiCreateStereoDisparityEstimator(VPI_BACKEND_CUDA, ctx->small_w, ctx->small_h, VPI_IMAGE_FORMAT_U8, &createParams, &ctx->stereoPayload));

        return ctx;
    }

    void compute_frame(PipelineContext* ctx, uint8_t* left_ptr, uint8_t* right_ptr, uint16_t* out_ptr) {
        VPIImageData data;
        memset(&data, 0, sizeof(data));
        data.format = VPI_IMAGE_FORMAT_U8;
        data.numPlanes = 1;
        data.planes[0].width = ctx->full_w;
        data.planes[0].height = ctx->full_h;
        data.planes[0].pitchBytes = ctx->full_w;
        
        data.planes[0].data = left_ptr;
        CHECK_STATUS(vpiImageSetWrappedHostMem(ctx->leftFull, &data));

        data.planes[0].data = right_ptr;
        CHECK_STATUS(vpiImageSetWrappedHostMem(ctx->rightFull, &data));

        CHECK_STATUS(vpiSubmitRescale(ctx->stream, VPI_BACKEND_CUDA, ctx->leftFull, ctx->leftSmall, VPI_INTERP_LINEAR, VPI_BORDER_CLAMP, 0));
        CHECK_STATUS(vpiSubmitRescale(ctx->stream, VPI_BACKEND_CUDA, ctx->rightFull, ctx->rightSmall, VPI_INTERP_LINEAR, VPI_BORDER_CLAMP, 0));

        VPIStereoDisparityEstimatorParams params;
        CHECK_STATUS(vpiInitStereoDisparityEstimatorParams(&params));
        
        params.windowSize = 11; 
        params.confidenceThreshold = 30000; 
        
        CHECK_STATUS(vpiSubmitStereoDisparityEstimator(ctx->stream, 0, ctx->stereoPayload, ctx->leftSmall, ctx->rightSmall, ctx->dispSmall, NULL, &params));
        CHECK_STATUS(vpiStreamSync(ctx->stream));
        
        VPIImageData outData;
        CHECK_STATUS(vpiImageLock(ctx->dispSmall, VPI_LOCK_READ, &outData));
        
        uint8_t* src = (uint8_t*)outData.planes[0].data;
        uint8_t* dst = (uint8_t*)out_ptr;
        int rowBytes = ctx->small_w * sizeof(uint16_t);
        
        for(int y=0; y<ctx->small_h; ++y) {
            memcpy(dst + y*rowBytes, src + y*outData.planes[0].pitchBytes, rowBytes);
        }
        CHECK_STATUS(vpiImageUnlock(ctx->dispSmall));
    }
}