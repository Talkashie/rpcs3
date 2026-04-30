#include "stdafx.h"
#include "nv406e.h"
#include "nv47_sync.hpp"

#include "Emu/RSX/RSXThread.h"

#include "context_accessors.define.h"

namespace rsx
{
	namespace nv406e
	{
		void set_reference(context* ctx, u32 /*reg*/, u32 arg)
		{
			RSX(ctx)->sync();

			// Write ref+get (get will be written again with the same value at command end)
			auto& dma = *vm::_ptr<RsxDmaControl>(RSX(ctx)->dma_address);
			dma.get.release(RSX(ctx)->fifo_ctrl->get_pos());
			dma.ref.store(arg);
		}

		void semaphore_acquire(context* ctx, u32, u32 arg)
		{
			const u32 addr = RSX(ctx)->label_addr + REGS(ctx)->registers[NV406E_SEMAPHORE_OFFSET];
			auto& sema = vm::_ref<atomic_t<RsxSemaphore>>(addr);

			auto flush_pending_semaphore_writes = [&]()
			{
				rsx::mm_flush();
			};

			if (sema != arg)
			{
				flush_pending_semaphore_writes();
			}

			u64 start = rsx::uclock();
			u64 last_check_val = start;
			u64 last_flush_val = start;

			while (sema != arg)
			{
				if (Emu.IsStopped())
				{
					break;
				}

				const u64 current = rsx::uclock();

				// Retry periodically while waiting. This covers cases where the release
				// is queued or delayed when the acquire first starts polling.
				if (current - last_flush_val > 1'000)
				{
					flush_pending_semaphore_writes();
					last_flush_val = current;
				}

				if (const auto tdr = static_cast<u64>(g_cfg.video.driver_recovery_timeout))
				{
					if (current - last_check_val > 20'000)
					{
						// Suspicious amount of time has passed.
						last_check_val = current;

						if (current - start > tdr * 1'000'000)
						{
							rsx_log.error("nv406e::semaphore_acquire has timed out. semaphore_address=0x%x", addr);
							break;
						}
					}
				}

				thread_ctrl::wait_for(50);
			}
		}

		void semaphore_release(context* ctx, u32 reg, u32 arg)
		{
			const u32 offset = REGS(ctx)->semaphore_offset_406e();

			if (offset % 4)
			{
				rsx_log.warning("NV406E semaphore release is using unaligned semaphore, ignoring. (offset=0x%x)", offset);
				return;
			}

			const u32 ctxt = REGS(ctx)->semaphore_context_dma_406e();

			// By avoiding doing this on flip's semaphore release
			// We allow last gcm's registers reset to occur in case of a crash
			if (const bool is_flip_sema = (offset == 0x10 && ctxt == CELL_GCM_CONTEXT_DMA_SEMAPHORE_R);
				!is_flip_sema)
			{
				RSX(ctx)->sync_point_request.release(true);
			}

			const u32 addr = get_address(offset, ctxt);

			// TODO: Check if possible to write on reservations
			if (RSX(ctx)->label_addr >> 28 != addr >> 28)
			{
				rsx_log.error("NV406E semaphore unexpected address. Please report to the developers. (offset=0x%x, addr=0x%x)", offset, addr);
				RSX(ctx)->recover_fifo();
				return;
			}

			if (addr == RSX(ctx)->device_addr + 0x30 && !arg)
			{
				// HW flip synchronization related, 1 is not written without display queue command (TODO: make it behave as real hw)
				arg = 1;
			}

			util::write_gcm_label<false, true>(ctx, reg, addr, arg);
		}
	}
}
