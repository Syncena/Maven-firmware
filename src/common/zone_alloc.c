/*
 * FreeRTOS Kernel V10.4.4
 * Copyright (C) 2021 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 */

/*
 * Copyright (c) 2022, Steve C. Woodford.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * A sample implementation of pvPortMalloc() and vPortFree() that combines
 * (coalescences) adjacent memory blocks as they are freed, and in so doing
 * limits memory fragmentation.
 *
 * See heap_1.c, heap_2.c and heap_3.c for alternative implementations, and the
 * memory management pages of https://www.FreeRTOS.org for more information.
 *
 * Modified by Steve Woodford to support multiple heaps, or "zones".
 */
#include <stdlib.h>
#include <string.h>

/* Defining MPU_WRAPPERS_INCLUDED_FROM_API_FILE prevents task.h from redefining
 * all the API functions to use the MPU wrappers.  That should only be done when
 * task.h is included from an application file. */
#define MPU_WRAPPERS_INCLUDED_FROM_API_FILE

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#undef MPU_WRAPPERS_INCLUDED_FROM_API_FILE

#include "zone_alloc.h"
#include "linked-lists.h"
#include "shell.h"

#if ( configSUPPORT_DYNAMIC_ALLOCATION == 0 )
    #error This file must not be used if configSUPPORT_DYNAMIC_ALLOCATION is 0
#endif

/* Block sizes must not get too small. */
#define heapMINIMUM_BLOCK_SIZE    ( ( size_t ) ( xHeapStructSize << 1 ) )

/* Assumes 8bit bytes! */
#define heapBITS_PER_BYTE         ( ( size_t ) 8 )

/* Define the linked list structure.  This is used to link free blocks in order
 * of their memory address. */
typedef struct A_BLOCK_LINK
{
    struct A_BLOCK_LINK * pxNextFreeBlock; /*<< The next free block in the list. */
    size_t xBlockSize;                     /*<< The size of the free block. */
} BlockLink_t;

/*-----------------------------------------------------------*/
typedef struct A_ZONE
{
    /* Mutex for this zone. If NULL, use vTaskSuspendAll(). */
    SemaphoreHandle_t xSemaphore;

    /* Create a couple of list links to mark the start and end of the list. */
    BlockLink_t xStart;
    BlockLink_t *pxEnd;

    /* Keeps track of the number of calls to allocate and free memory as well as the
     * number of free bytes remaining, but says nothing about fragmentation. */
    size_t xFreeBytesRemaining;
    size_t xMinimumEverFreeBytesRemaining;
    size_t xNumberOfSuccessfulAllocations;
    size_t xNumberOfSuccessfulFrees;

    /* Gets set to the top bit of an size_t type.  When this bit in the xBlockSize
     * member of an BlockLink_t structure is set then the block belongs to the
     * application.  When the bit is free the block is still part of the free heap
     * space. */
    size_t xBlockAllocatedBit;

    const char *pcName;

#if (RELEASE_BUILD == 0)
    size_t xInitialLen;
    TAILQ_ENTRY(A_ZONE) xQEnt;
#endif
} *Zone_t;

#if (RELEASE_BUILD == 0)
TAILQ_HEAD(A_ZONE_QHEAD, A_ZONE);
static struct A_ZONE_QHEAD xZoneList = TAILQ_HEAD_INITIALIZER(xZoneList);
#endif

/*-----------------------------------------------------------*/
/*
 * Inserts a block of memory that is being freed into the correct position in
 * the list of free memory blocks.  The block being freed will be merged with
 * the block in front it and/or the block behind it if the memory blocks are
 * adjacent to each other.
 */
static void prvInsertBlockIntoFreeList( Zone_t xZone, BlockLink_t * pxBlockToInsert ) PRIVILEGED_FUNCTION;

/*-----------------------------------------------------------*/

/* The size of the structure placed at the beginning of each allocated memory
 * block must by correctly byte aligned. */
static const size_t xHeapStructSize = ( sizeof( BlockLink_t ) + ( ( size_t ) ( portBYTE_ALIGNMENT - 1 ) ) ) & ~( ( size_t ) portBYTE_ALIGNMENT_MASK );

PRIVILEGED_DATA static Zone_t prxHeapZone = NULL;

#if (RELEASE_BUILD == 0)
SHELL_CMD_DECL(heap, heap_cmd, "Show heap usage stats");
#endif

/*-----------------------------------------------------------*/
static void vLockZone( Zone_t xZone )
{
    if ( xZone->xSemaphore != NULL )
    {
        xSemaphoreTake( xZone->xSemaphore, portMAX_DELAY );
    }
    else
    {
        vTaskSuspendAll();
    }
}

static void vUnLockZone( Zone_t xZone )
{
    if ( xZone->xSemaphore != NULL )
    {
        xSemaphoreGive( xZone->xSemaphore );
    }
    else
    {
        (void) xTaskResumeAll();
    }
}

static void * pvMalloc( Zone_t xZone, size_t xWantedSize )
{
    BlockLink_t * pxBlock, * pxPreviousBlock, * pxNewBlockLink;
    void * pvReturn = NULL;

    vLockZone( xZone );
    {
        /* Check the requested block size is not so large that the top bit is
         * set.  The top bit of the block size member of the BlockLink_t structure
         * is used to determine who owns the block - the application or the
         * kernel, so it must be free. */
        if( ( xWantedSize & xZone->xBlockAllocatedBit ) == 0 )
        {
            /* The wanted size must be increased so it can contain a BlockLink_t
             * structure in addition to the requested amount of bytes. */
            if( ( xWantedSize > 0 ) &&
                ( ( xWantedSize + xHeapStructSize ) >  xWantedSize ) ) /* Overflow check */
            {
                xWantedSize += xHeapStructSize;

                /* Ensure that blocks are always aligned. */
                if( ( xWantedSize & portBYTE_ALIGNMENT_MASK ) != 0x00 )
                {
                    /* Byte alignment required. Check for overflow. */
                    if( ( xWantedSize + ( portBYTE_ALIGNMENT - ( xWantedSize & portBYTE_ALIGNMENT_MASK ) ) )
                            > xWantedSize )
                    {
                        xWantedSize += ( portBYTE_ALIGNMENT - ( xWantedSize & portBYTE_ALIGNMENT_MASK ) );
                        configASSERT( ( xWantedSize & portBYTE_ALIGNMENT_MASK ) == 0 );
                    }
                    else
                    {
                        xWantedSize = 0;
                    }
                }
                else
                {
                    mtCOVERAGE_TEST_MARKER();
                }
            }
            else
            {
                xWantedSize = 0;
            }

            if( ( xWantedSize > 0 ) && ( xWantedSize <= xZone->xFreeBytesRemaining ) )
            {
                /* Traverse the list from the start (lowest address) block until
                 * one of adequate size is found. */
                pxPreviousBlock = &xZone->xStart;
                pxBlock = xZone->xStart.pxNextFreeBlock;

                while( ( pxBlock->xBlockSize < xWantedSize ) && ( pxBlock->pxNextFreeBlock != NULL ) )
                {
                    pxPreviousBlock = pxBlock;
                    pxBlock = pxBlock->pxNextFreeBlock;
                }

                /* If the end marker was reached then a block of adequate size
                 * was not found. */
                if( pxBlock != xZone->pxEnd )
                {
                    /* Return the memory space pointed to - jumping over the
                     * BlockLink_t structure at its start. */
                    pvReturn = ( void * ) ( ( ( uint8_t * ) pxPreviousBlock->pxNextFreeBlock ) + xHeapStructSize );

                    /* This block is being returned for use so must be taken out
                     * of the list of free blocks. */
                    pxPreviousBlock->pxNextFreeBlock = pxBlock->pxNextFreeBlock;

                    /* If the block is larger than required it can be split into
                     * two. */
                    if( ( pxBlock->xBlockSize - xWantedSize ) > heapMINIMUM_BLOCK_SIZE )
                    {
                        /* This block is to be split into two.  Create a new
                         * block following the number of bytes requested. The void
                         * cast is used to prevent byte alignment warnings from the
                         * compiler. */
                        pxNewBlockLink = ( void * ) ( ( ( uint8_t * ) pxBlock ) + xWantedSize );
                        configASSERT( ( ( ( size_t ) pxNewBlockLink ) & portBYTE_ALIGNMENT_MASK ) == 0 );

                        /* Calculate the sizes of two blocks split from the
                         * single block. */
                        pxNewBlockLink->xBlockSize = pxBlock->xBlockSize - xWantedSize;
                        pxBlock->xBlockSize = xWantedSize;

                        /* Insert the new block into the list of free blocks. */
                        prvInsertBlockIntoFreeList( xZone, pxNewBlockLink );
                    }
                    else
                    {
                        mtCOVERAGE_TEST_MARKER();
                    }

                    xZone->xFreeBytesRemaining -= pxBlock->xBlockSize;

                    if( xZone->xFreeBytesRemaining < xZone->xMinimumEverFreeBytesRemaining )
                    {
                        xZone->xMinimumEverFreeBytesRemaining = xZone->xFreeBytesRemaining;
                    }
                    else
                    {
                        mtCOVERAGE_TEST_MARKER();
                    }

                    /* The block is being returned - it is allocated and owned
                     * by the application and has no "next" block. */
                    pxBlock->xBlockSize |= xZone->xBlockAllocatedBit;
                    pxBlock->pxNextFreeBlock = NULL;
                    xZone->xNumberOfSuccessfulAllocations++;
                }
                else
                {
                    mtCOVERAGE_TEST_MARKER();
                }
            }
            else
            {
                mtCOVERAGE_TEST_MARKER();
            }
        }
        else
        {
            mtCOVERAGE_TEST_MARKER();
        }

        traceMALLOC( pvReturn, xWantedSize );
    }
    vUnLockZone( xZone );

    #if ( configUSE_MALLOC_FAILED_HOOK == 1 )
        {
            if( pvReturn == NULL )
            {
                extern void vApplicationMallocFailedHook( void );
                vApplicationMallocFailedHook();
            }
            else
            {
                mtCOVERAGE_TEST_MARKER();
            }
        }
    #endif /* if ( configUSE_MALLOC_FAILED_HOOK == 1 ) */

    configASSERT( ( ( ( size_t ) pvReturn ) & ( size_t ) portBYTE_ALIGNMENT_MASK ) == 0 );
    return pvReturn;
}
/*-----------------------------------------------------------*/

static void vFree( Zone_t xZone, void * pv )
{
    uint8_t * puc = ( uint8_t * ) pv;
    BlockLink_t * pxLink;

    if( pv != NULL )
    {
        /* The memory being freed will have an BlockLink_t structure immediately
         * before it. */
        puc -= xHeapStructSize;

        /* This casting is to keep the compiler from issuing warnings. */
        pxLink = ( void * ) puc;

        /* Check the block is actually allocated. */
        configASSERT( ( pxLink->xBlockSize & xZone->xBlockAllocatedBit ) != 0 );
        configASSERT( pxLink->pxNextFreeBlock == NULL );

        if( ( pxLink->xBlockSize & xZone->xBlockAllocatedBit ) != 0 )
        {
            if( pxLink->pxNextFreeBlock == NULL )
            {
                /* The block is being returned to the heap - it is no longer
                 * allocated. */
                pxLink->xBlockSize &= ~xZone->xBlockAllocatedBit;

                vLockZone( xZone );
                {
                    /* Add this block to the list of free blocks. */
                    xZone->xFreeBytesRemaining += pxLink->xBlockSize;
                    traceFREE( pv, pxLink->xBlockSize );
                    prvInsertBlockIntoFreeList( xZone, ( ( BlockLink_t * ) pxLink ) );
                    xZone->xNumberOfSuccessfulFrees++;
                }
                vUnLockZone( xZone );
            }
            else
            {
                mtCOVERAGE_TEST_MARKER();
            }
        }
        else
        {
            mtCOVERAGE_TEST_MARKER();
        }
    }
}
/*-----------------------------------------------------------*/

size_t xPortGetFreeHeapSize( void )
{
    return prxHeapZone->xFreeBytesRemaining;
}
/*-----------------------------------------------------------*/

size_t xPortGetMinimumEverFreeHeapSize( void )
{
    return prxHeapZone->xMinimumEverFreeBytesRemaining;
}
/*-----------------------------------------------------------*/

void vPortInitialiseBlocks( void )
{
    /* This just exists to keep the linker quiet. */
}
/*-----------------------------------------------------------*/

static Zone_t vHeapInit( SemaphoreHandle_t xSemaphore, void *vMemory, size_t xMemoryLen )
{
    BlockLink_t * pxFirstFreeBlock;
    uint8_t * pucAlignedHeap;
    size_t uxAddress;
    Zone_t xZone;

    /* Ensure the heap starts on a correctly aligned boundary. */
    uxAddress = ( size_t ) vMemory;

    if( ( uxAddress & portBYTE_ALIGNMENT_MASK ) != 0 )
    {
        uxAddress += ( portBYTE_ALIGNMENT - 1 );
        uxAddress &= ~( ( size_t ) portBYTE_ALIGNMENT_MASK );
    }

    /* Allocate space for the Zone_t from the start of the memory region. */
    xZone = ( Zone_t )( uintptr_t) uxAddress;
#if (RELEASE_BUILD == 0)
    xZone->xInitialLen = xMemoryLen;
#endif

    /* Account for the Zone_t and alignment. */
    uxAddress += sizeof( *xZone );
    uxAddress += ( portBYTE_ALIGNMENT - 1 );
    uxAddress &= ~( ( size_t ) portBYTE_ALIGNMENT_MASK );

    /* Calculate remaining heap size. */
    xMemoryLen -= uxAddress - ( size_t ) vMemory;

    pucAlignedHeap = ( uint8_t * ) uxAddress;

    /* xStart is used to hold a pointer to the first item in the list of free
     * blocks.  The void cast is used to prevent compiler warnings. */
    xZone->xStart.pxNextFreeBlock = ( void * ) pucAlignedHeap;
    xZone->xStart.xBlockSize = ( size_t ) 0;

    /* pxEnd is used to mark the end of the list of free blocks and is inserted
     * at the end of the heap space. */
    uxAddress = ( ( size_t ) pucAlignedHeap ) + xMemoryLen;
    uxAddress -= xHeapStructSize;
    uxAddress &= ~( ( size_t ) portBYTE_ALIGNMENT_MASK );
    xZone->pxEnd = ( void * ) uxAddress;
    xZone->pxEnd->xBlockSize = 0;
    xZone->pxEnd->pxNextFreeBlock = NULL;

    /* To start with there is a single free block that is sized to take up the
     * entire heap space, minus the space taken by pxEnd. */
    pxFirstFreeBlock = ( void * ) pucAlignedHeap;
    pxFirstFreeBlock->xBlockSize = uxAddress - ( size_t ) pxFirstFreeBlock;
    pxFirstFreeBlock->pxNextFreeBlock = xZone->pxEnd;

    /* Only one block exists - and it covers the entire usable heap space. */
    xZone->xMinimumEverFreeBytesRemaining = pxFirstFreeBlock->xBlockSize;
    xZone->xFreeBytesRemaining = pxFirstFreeBlock->xBlockSize;

    /* Work out the position of the top bit in a size_t variable. */
    xZone->xBlockAllocatedBit = ( ( size_t ) 1 ) << ( ( sizeof( size_t ) * heapBITS_PER_BYTE ) - 1 );

    xZone->xSemaphore = xSemaphore;
    xZone->xNumberOfSuccessfulAllocations = 0;
    xZone->xNumberOfSuccessfulFrees = 0;
    xZone->pcName = NULL;

    return xZone;
}
/*-----------------------------------------------------------*/

static void prvInsertBlockIntoFreeList( Zone_t xZone, BlockLink_t * pxBlockToInsert ) /* PRIVILEGED_FUNCTION */
{
    BlockLink_t * pxIterator;
    uint8_t * puc;

    /* Iterate through the list until a block is found that has a higher address
     * than the block being inserted. */
    for( pxIterator = &xZone->xStart; pxIterator->pxNextFreeBlock < pxBlockToInsert; pxIterator = pxIterator->pxNextFreeBlock )
    {
        /* Nothing to do here, just iterate to the right position. */
    }

    /* Do the block being inserted, and the block it is being inserted after
     * make a contiguous block of memory? */
    puc = ( uint8_t * ) pxIterator;

    if( ( puc + pxIterator->xBlockSize ) == ( uint8_t * ) pxBlockToInsert )
    {
        pxIterator->xBlockSize += pxBlockToInsert->xBlockSize;
        pxBlockToInsert = pxIterator;
    }
    else
    {
        mtCOVERAGE_TEST_MARKER();
    }

    /* Do the block being inserted, and the block it is being inserted before
     * make a contiguous block of memory? */
    puc = ( uint8_t * ) pxBlockToInsert;

    if( ( puc + pxBlockToInsert->xBlockSize ) == ( uint8_t * ) pxIterator->pxNextFreeBlock )
    {
        if( pxIterator->pxNextFreeBlock != xZone->pxEnd )
        {
            /* Form one big block from the two blocks. */
            pxBlockToInsert->xBlockSize += pxIterator->pxNextFreeBlock->xBlockSize;
            pxBlockToInsert->pxNextFreeBlock = pxIterator->pxNextFreeBlock->pxNextFreeBlock;
        }
        else
        {
            pxBlockToInsert->pxNextFreeBlock = xZone->pxEnd;
        }
    }
    else
    {
        pxBlockToInsert->pxNextFreeBlock = pxIterator->pxNextFreeBlock;
    }

    /* If the block being inserted plugged a gab, so was merged with the block
     * before and the block after, then it's pxNextFreeBlock pointer will have
     * already been set, and should not be set here as that would make it point
     * to itself. */
    if( pxIterator != pxBlockToInsert )
    {
        pxIterator->pxNextFreeBlock = pxBlockToInsert;
    }
    else
    {
        mtCOVERAGE_TEST_MARKER();
    }
}
/*-----------------------------------------------------------*/
#if (RELEASE_BUILD == 0)
static void vGetHeapStats( Zone_t xZone, HeapStats_t * pxHeapStats )
{
    BlockLink_t * pxBlock;
    size_t xBlocks = 0, xMaxSize = 0, xMinSize = portMAX_DELAY; /* portMAX_DELAY used as a portable way of getting the maximum value. */

    vLockZone( xZone );
    {
        pxBlock = xZone->xStart.pxNextFreeBlock;

        /* pxBlock will be NULL if the heap has not been initialised.  The heap
         * is initialised automatically when the first allocation is made. */
        if( pxBlock != NULL )
        {
            do
            {
                /* Increment the number of blocks and record the largest block seen
                 * so far. */
                xBlocks++;

                if( pxBlock->xBlockSize > xMaxSize )
                {
                    xMaxSize = pxBlock->xBlockSize;
                }

                if( pxBlock->xBlockSize < xMinSize )
                {
                    xMinSize = pxBlock->xBlockSize;
                }

                /* Move to the next block in the chain until the last block is
                 * reached. */
                pxBlock = pxBlock->pxNextFreeBlock;
            } while( pxBlock != xZone->pxEnd );
        }
    }
    vUnLockZone( xZone );

    pxHeapStats->xSizeOfLargestFreeBlockInBytes = xMaxSize;
    pxHeapStats->xSizeOfSmallestFreeBlockInBytes = xMinSize;
    pxHeapStats->xNumberOfFreeBlocks = xBlocks;

    taskENTER_CRITICAL();
    {
        pxHeapStats->xAvailableHeapSpaceInBytes = xZone->xFreeBytesRemaining;
        pxHeapStats->xNumberOfSuccessfulAllocations = xZone->xNumberOfSuccessfulAllocations;
        pxHeapStats->xNumberOfSuccessfulFrees = xZone->xNumberOfSuccessfulFrees;
        pxHeapStats->xMinimumEverFreeBytesRemaining = xZone->xMinimumEverFreeBytesRemaining;
    }
    taskEXIT_CRITICAL();
}

static void
heap_cmd(FILE *os, uint8_t argc, const char * const *argv)
{
	HeapStats_t hs;
	Zone_t zone;

	(void) argc;
	(void) argv;

	TAILQ_FOREACH(zone, &xZoneList, xQEnt) {
		vGetHeapStats(zone, &hs);
		fprintf(os, "Zone @ %p: %s\n\tSize %u, Free size: %u "
		    "(max %u, min %u)\n", (void *)zone, zone->pcName,
		    (unsigned int)zone->xInitialLen,
		    (unsigned int)hs.xAvailableHeapSpaceInBytes,
		    (unsigned int)hs.xSizeOfLargestFreeBlockInBytes,
		    (unsigned int)hs.xSizeOfSmallestFreeBlockInBytes);
		fprintf(os, "\tFree chunks: %u, Allocs: %u, Frees %u\n",
		    (unsigned int)hs.xNumberOfFreeBlocks,
		    (unsigned int)hs.xNumberOfSuccessfulAllocations,
		    (unsigned int)hs.xNumberOfSuccessfulFrees);
	}
}
#endif	/* RELEASE_BUILD == 0 */

/*
 * Public zone API
 */
zone_t
zone_create(void *m, void *mem, size_t len, int flags)
{
	SemaphoreHandle_t mutex = m;
	Zone_t zone = vHeapInit(mutex, mem, len);
#if (RELEASE_BUILD == 0)
	bool add_cmd = false;
#endif

	(void) flags;

        vTaskSuspendAll();
	if (prxHeapZone == NULL) {
		zone->pcName = "HEAP";
		prxHeapZone = zone;
#if (RELEASE_BUILD == 0)
		add_cmd = true;
#endif
	}
#if (RELEASE_BUILD == 0)
	TAILQ_INSERT_TAIL(&xZoneList, zone, xQEnt);
#endif
        (void) xTaskResumeAll();

#if (RELEASE_BUILD == 0)
	if (add_cmd)
		SHELL_CMD_ADD(heap);
#endif

	return (zone_t)zone;
}

zone_t
zone_create_named(const char *name, void *m, void *mem, size_t len, int flags)
{
	SemaphoreHandle_t mutex = m;
	Zone_t zone = vHeapInit(mutex, mem, len);

	(void) flags;

	zone->pcName = name;

#if (RELEASE_BUILD == 0)
        vTaskSuspendAll();
	TAILQ_INSERT_TAIL(&xZoneList, zone, xQEnt);
        (void) xTaskResumeAll();
#endif

	return (zone_t)zone;
}

void
zone_destroy(zone_t z)
{
#if (RELEASE_BUILD == 0)
	Zone_t zone = (Zone_t)z;

        vTaskSuspendAll();
	TAILQ_REMOVE(&xZoneList, zone, xQEnt);
        (void) xTaskResumeAll();
#else
	(void) z;
#endif
}

void *
zone_malloc(size_t len)
{

	return pvMalloc(prxHeapZone, len);
}

void *
zone_calloc(size_t nitems, size_t itemlen)
{
	size_t len = nitems * itemlen;
	void *p = zone_malloc(len);

	memset(p, 0, len);

	return p;
}

void
zone_free(void *p)
{

	vFree(prxHeapZone, p);
}

char *
zone_strdup(const char *str)
{
	size_t len = strlen(str) + 1;
	void *p = zone_malloc(len);

	memcpy(p, str, len);

	return p;
}

void *
zone_malloc_private(zone_t z, size_t len)
{

	return pvMalloc((Zone_t)z, len);
}

void *
zone_calloc_private(zone_t z, size_t nitems, size_t itemlen)
{
	size_t len = nitems * itemlen;
	void *p = pvMalloc((Zone_t)z, len);

	memset(p, 0, len);

	return p;
}

void
zone_free_private(zone_t z, void *p)
{

	vFree((Zone_t)z, p);
}
