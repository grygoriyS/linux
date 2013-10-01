/*
 * Discontiguous memory support, Kanoj Sarcar, SGI, Nov 1999
 */
#ifndef _LINUX_BOOTMEM_H
#define _LINUX_BOOTMEM_H

#include <linux/mmzone.h>
#include <asm/dma.h>

/*
 *  simple boot-time physical memory area allocator.
 */

extern unsigned long max_low_pfn;
extern unsigned long min_low_pfn;

/*
 * highest page
 */
extern unsigned long max_pfn;

#ifndef CONFIG_NO_BOOTMEM
/*
 * node_bootmem_map is a map pointer - the bits represent all physical 
 * memory pages (including holes) on the node.
 */
typedef struct bootmem_data {
	unsigned long node_min_pfn;
	unsigned long node_low_pfn;
	void *node_bootmem_map;
	unsigned long last_end_off;
	unsigned long hint_idx;
	struct list_head list;
} bootmem_data_t;

extern bootmem_data_t bootmem_node_data[];
#endif

extern unsigned long bootmem_bootmap_pages(unsigned long);

extern unsigned long init_bootmem_node(pg_data_t *pgdat,
				       unsigned long freepfn,
				       unsigned long startpfn,
				       unsigned long endpfn);
extern unsigned long init_bootmem(unsigned long addr, unsigned long memend);

extern unsigned long free_all_bootmem(void);
extern void reset_all_zones_managed_pages(void);

extern void free_bootmem_node(pg_data_t *pgdat,
			      unsigned long addr,
			      unsigned long size);
extern void free_bootmem(unsigned long physaddr, unsigned long size);
extern void free_bootmem_late(unsigned long physaddr, unsigned long size);

/*
 * Flags for reserve_bootmem (also if CONFIG_HAVE_ARCH_BOOTMEM_NODE,
 * the architecture-specific code should honor this).
 *
 * If flags is 0, then the return value is always 0 (success). If
 * flags contains BOOTMEM_EXCLUSIVE, then -EBUSY is returned if the
 * memory already was reserved.
 */
#define BOOTMEM_DEFAULT		0
#define BOOTMEM_EXCLUSIVE	(1<<0)

extern int reserve_bootmem(unsigned long addr,
			   unsigned long size,
			   int flags);
extern int reserve_bootmem_node(pg_data_t *pgdat,
				unsigned long physaddr,
				unsigned long size,
				int flags);

extern void *__alloc_bootmem(unsigned long size,
			     unsigned long align,
			     unsigned long goal);
extern void *__alloc_bootmem_nopanic(unsigned long size,
				     unsigned long align,
				     unsigned long goal);
extern void *__alloc_bootmem_node(pg_data_t *pgdat,
				  unsigned long size,
				  unsigned long align,
				  unsigned long goal);
void *__alloc_bootmem_node_high(pg_data_t *pgdat,
				  unsigned long size,
				  unsigned long align,
				  unsigned long goal);
extern void *__alloc_bootmem_node_nopanic(pg_data_t *pgdat,
				  unsigned long size,
				  unsigned long align,
				  unsigned long goal);
void *___alloc_bootmem_node_nopanic(pg_data_t *pgdat,
				  unsigned long size,
				  unsigned long align,
				  unsigned long goal,
				  unsigned long limit);
extern void *__alloc_bootmem_low(unsigned long size,
				 unsigned long align,
				 unsigned long goal);
void *__alloc_bootmem_low_nopanic(unsigned long size,
				 unsigned long align,
				 unsigned long goal);
extern void *__alloc_bootmem_low_node(pg_data_t *pgdat,
				      unsigned long size,
				      unsigned long align,
				      unsigned long goal);

#ifdef CONFIG_NO_BOOTMEM
/* We are using top down, so it is safe to use 0 here */
#define BOOTMEM_LOW_LIMIT 0
#else
#define BOOTMEM_LOW_LIMIT __pa(MAX_DMA_ADDRESS)
#endif

#define alloc_bootmem(x) \
	__alloc_bootmem(x, SMP_CACHE_BYTES, BOOTMEM_LOW_LIMIT)
#define alloc_bootmem_align(x, align) \
	__alloc_bootmem(x, align, BOOTMEM_LOW_LIMIT)
#define alloc_bootmem_nopanic(x) \
	__alloc_bootmem_nopanic(x, SMP_CACHE_BYTES, BOOTMEM_LOW_LIMIT)
#define alloc_bootmem_pages(x) \
	__alloc_bootmem(x, PAGE_SIZE, BOOTMEM_LOW_LIMIT)
#define alloc_bootmem_pages_nopanic(x) \
	__alloc_bootmem_nopanic(x, PAGE_SIZE, BOOTMEM_LOW_LIMIT)
#define alloc_bootmem_node(pgdat, x) \
	__alloc_bootmem_node(pgdat, x, SMP_CACHE_BYTES, BOOTMEM_LOW_LIMIT)
#define alloc_bootmem_node_nopanic(pgdat, x) \
	__alloc_bootmem_node_nopanic(pgdat, x, SMP_CACHE_BYTES, BOOTMEM_LOW_LIMIT)
#define alloc_bootmem_pages_node(pgdat, x) \
	__alloc_bootmem_node(pgdat, x, PAGE_SIZE, BOOTMEM_LOW_LIMIT)
#define alloc_bootmem_pages_node_nopanic(pgdat, x) \
	__alloc_bootmem_node_nopanic(pgdat, x, PAGE_SIZE, BOOTMEM_LOW_LIMIT)

#define alloc_bootmem_low(x) \
	__alloc_bootmem_low(x, SMP_CACHE_BYTES, 0)
#define alloc_bootmem_low_pages_nopanic(x) \
	__alloc_bootmem_low_nopanic(x, PAGE_SIZE, 0)
#define alloc_bootmem_low_pages(x) \
	__alloc_bootmem_low(x, PAGE_SIZE, 0)
#define alloc_bootmem_low_pages_node(pgdat, x) \
	__alloc_bootmem_low_node(pgdat, x, PAGE_SIZE, 0)


#if defined(CONFIG_HAVE_MEMBLOCK) && defined(CONFIG_NO_BOOTMEM)

/* FIXME: use MEMBLOCK_ALLOC_* variants here */
#define BOOTMEM_ALLOC_ACCESSIBLE	0
#define BOOTMEM_ALLOC_ANYWHERE		(~(phys_addr_t)0)

/*
 * FIXME: use NUMA_NO_NODE instead of MAX_NUMNODES when bootmem/nobootmem code
 * will be removed.
 * It can't be done now, because when MEMBLOCK or NO_BOOTMEM are not enabled
 * all calls of the new API will be redirected to bottmem/nobootmem where
 * MAX_NUMNODES is widely used.
 * Also, memblock core APIs __next_free_mem_range_rev() and
 * __next_free_mem_range() would need to be updated, and as result we will
 * need to re-check/update all direct calls of memblock_alloc_xxx()
 * APIs (including nobootmem).
 */

/* FIXME: Move to memblock.h at a point where we remove nobootmem.c */
void *memblock_virt_alloc_try_nid_nopanic(phys_addr_t size,
		phys_addr_t align, phys_addr_t from,
		phys_addr_t max_addr, int nid);
void *memblock_virt_alloc_try_nid(phys_addr_t size, phys_addr_t align,
		phys_addr_t from, phys_addr_t max_addr, int nid);
void __memblock_free_early(phys_addr_t base, phys_addr_t size);
void __memblock_free_late(phys_addr_t base, phys_addr_t size);

#define memblock_virt_alloc(x) \
	memblock_virt_alloc_try_nid(x, SMP_CACHE_BYTES, BOOTMEM_LOW_LIMIT, \
				     BOOTMEM_ALLOC_ACCESSIBLE, MAX_NUMNODES)
#define memblock_virt_alloc_align(x, align) \
	memblock_virt_alloc_try_nid(x, align, BOOTMEM_LOW_LIMIT, \
				     BOOTMEM_ALLOC_ACCESSIBLE, MAX_NUMNODES)
#define memblock_virt_alloc_nopanic(x) \
	memblock_virt_alloc_try_nid_nopanic(x, SMP_CACHE_BYTES, \
					     BOOTMEM_LOW_LIMIT, \
					     BOOTMEM_ALLOC_ACCESSIBLE, \
					     MAX_NUMNODES)
#define memblock_virt_alloc_align_nopanic(x, align) \
	memblock_virt_alloc_try_nid_nopanic(x, align, \
					     BOOTMEM_LOW_LIMIT, \
					     BOOTMEM_ALLOC_ACCESSIBLE, \
					     MAX_NUMNODES)
#define memblock_virt_alloc_node(x, nid) \
	memblock_virt_alloc_try_nid(x, SMP_CACHE_BYTES, BOOTMEM_LOW_LIMIT, \
				     BOOTMEM_ALLOC_ACCESSIBLE, nid)
#define memblock_virt_alloc_node_nopanic(x, nid) \
	memblock_virt_alloc_try_nid_nopanic(x, SMP_CACHE_BYTES, \
					     BOOTMEM_LOW_LIMIT, \
					     BOOTMEM_ALLOC_ACCESSIBLE, nid)

#define memblock_free_early(x, s)		__memblock_free_early(x, s)
#define memblock_free_early_nid(x, s, nid)	__memblock_free_early(x, s)
#define memblock_free_late(x, s)		__memblock_free_late(x, s)

#else

/* Fall back to all the existing bootmem APIs */
#define memblock_virt_alloc(x) \
	__alloc_bootmem(x, SMP_CACHE_BYTES, BOOTMEM_LOW_LIMIT)
#define memblock_virt_alloc_align(x, align) \
	__alloc_bootmem(x, align, BOOTMEM_LOW_LIMIT)
#define memblock_virt_alloc_nopanic(x) \
	__alloc_bootmem_nopanic(x, SMP_CACHE_BYTES, BOOTMEM_LOW_LIMIT)
#define memblock_virt_alloc_node(x, nid) \
	__alloc_bootmem_node(NODE_DATA(nid), x, SMP_CACHE_BYTES, \
			BOOTMEM_LOW_LIMIT)
#define memblock_virt_alloc_node_nopanic(x, nid) \
	__alloc_bootmem_node_nopanic(NODE_DATA(nid), x, SMP_CACHE_BYTES, \
			BOOTMEM_LOW_LIMIT)
#define memblock_virt_alloc_try_nid(size, align, from, max_addr, nid) \
		__alloc_bootmem_node_high(NODE_DATA(nid), size, align, from)
#define memblock_virt_alloc_try_nid_nopanic(size, align, from, max_addr, nid) \
		___alloc_bootmem_node_nopanic(NODE_DATA(nid), size, align, \
			from, max_addr)
#define memblock_free_early(x, s)	free_bootmem(x, s)
#define memblock_free_early_nid(x, s, nid) \
			free_bootmem_node(NODE_DATA(nid), x, s)
#define memblock_free_late(x, s)	free_bootmem_late(x, s)

#endif /* defined(CONFIG_HAVE_MEMBLOCK) && defined(CONFIG_NO_BOOTMEM) */

#ifdef CONFIG_HAVE_ARCH_ALLOC_REMAP
extern void *alloc_remap(int nid, unsigned long size);
#else
static inline void *alloc_remap(int nid, unsigned long size)
{
	return NULL;
}
#endif /* CONFIG_HAVE_ARCH_ALLOC_REMAP */

extern void *alloc_large_system_hash(const char *tablename,
				     unsigned long bucketsize,
				     unsigned long numentries,
				     int scale,
				     int flags,
				     unsigned int *_hash_shift,
				     unsigned int *_hash_mask,
				     unsigned long low_limit,
				     unsigned long high_limit);

#define HASH_EARLY	0x00000001	/* Allocating during early boot? */
#define HASH_SMALL	0x00000002	/* sub-page allocation allowed, min
					 * shift passed via *_hash_shift */

/* Only NUMA needs hash distribution. 64bit NUMA architectures have
 * sufficient vmalloc space.
 */
#if defined(CONFIG_NUMA) && defined(CONFIG_64BIT)
#define HASHDIST_DEFAULT 1
#else
#define HASHDIST_DEFAULT 0
#endif
extern int hashdist;		/* Distribute hashes across NUMA nodes? */


#endif /* _LINUX_BOOTMEM_H */
