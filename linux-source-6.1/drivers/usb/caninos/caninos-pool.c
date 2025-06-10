#include "aotg_hcd.h"

void aotg_hcd_pool_init(struct aotg_hcd *acthcd)
{
	int i;
	BUG_ON(!acthcd);
	
	for (i = 0; i < MAX_EP_NUM; i++)
	{
		WRITE_ONCE(acthcd->hcep_pool.ep0[i], NULL);
		WRITE_ONCE(acthcd->hcep_pool.inep[i], NULL);
		WRITE_ONCE(acthcd->hcep_pool.outep[i], NULL);
	}
	
	spin_lock_init(&acthcd->queue_pool.lock);
	
	atomic64_set_release(&acthcd->queue_pool.used_queue, 0LL);
}

extern struct aotg_queue *aotg_hcd_queue_alloc(struct aotg_hcd *acthcd)
{
	struct aotg_queue_pool *pool;
	struct aotg_queue *q;
	unsigned long flags;
	int i;
	
	BUG_ON(!acthcd);
	pool = &acthcd->queue_pool;
	spin_lock_irqsave(&pool->lock, flags);
	
	for (i = 0; i < AOTG_QUEUE_POOL_CNT; i++)
	{
		s64 mask = (s64) BIT_ULL(i);
		
		if (atomic64_fetch_or_acquire(mask, &pool->used_queue) & mask) {
			continue;
		}
		
		spin_unlock_irqrestore(&pool->lock, flags);
		
		q = &pool->queue[i];
		memset(q, 0, sizeof(*q));
		INIT_LIST_HEAD(&q->enqueue_list);
		INIT_LIST_HEAD(&q->dequeue_list);
		INIT_LIST_HEAD(&q->finished_list);
		return q;
	}
	
	spin_unlock_irqrestore(&pool->lock, flags);
	return NULL;
}

void aotg_hcd_queue_free(struct aotg_hcd *acthcd, struct aotg_queue *q)
{
	struct aotg_queue_pool *pool;
	unsigned long flags;
	int i;
	
	BUG_ON(!acthcd);
	pool = &acthcd->queue_pool;
	spin_lock_irqsave(&pool->lock, flags);
	
	if (!q) { /* release all */
		atomic64_set_release(&pool->used_queue, 0LL);
	}
	else for (i = 0; i < AOTG_QUEUE_POOL_CNT; i++)
	{
		if (q == &pool->queue[i])
		{
			s64 mask = (s64) BIT_ULL(i);
			atomic64_fetch_andnot_release(mask, &pool->used_queue);
			break;
		}
	}
	
	spin_unlock_irqrestore(&pool->lock, flags);
}

