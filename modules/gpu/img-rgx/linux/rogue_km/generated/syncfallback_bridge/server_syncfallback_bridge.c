/*******************************************************************************
@File
@Title          Server bridge for syncfallback
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    Implements the server side of the bridge for syncfallback
@License        Dual MIT/GPLv2

The contents of this file are subject to the MIT license as set out below.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

Alternatively, the contents of this file may be used under the terms of
the GNU General Public License Version 2 ("GPL") in which case the provisions
of GPL are applicable instead of those above.

If you wish to allow use of your version of this file only under the terms of
GPL, and not to allow others to use your version of this file under the terms
of the MIT license, indicate your decision by deleting the provisions above
and replace them with the notice and other provisions required by GPL as set
out in the file called "GPL-COPYING" included in this distribution. If you do
not delete the provisions above, a recipient may use your version of this file
under the terms of either the MIT license or GPL.

This License is also included in this distribution in the file called
"MIT-COPYING".

EXCEPT AS OTHERWISE STATED IN A NEGOTIATED AGREEMENT: (A) THE SOFTWARE IS
PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT; AND (B) IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
********************************************************************************/

#include <linux/uaccess.h>

#include "img_defs.h"

#include "sync_fallback_server.h"
#include "pvrsrv_sync_server.h"


#include "common_syncfallback_bridge.h"

#include "allocmem.h"
#include "pvr_debug.h"
#include "connection_server.h"
#include "pvr_bridge.h"
#if defined(SUPPORT_RGX)
#include "rgx_bridge.h"
#endif
#include "srvcore.h"
#include "handle.h"

#include <linux/slab.h>




#if defined(SUPPORT_INSECURE_EXPORT)
static PVRSRV_ERROR ReleaseExport(void *pvData)
{
	PVR_UNREFERENCED_PARAMETER(pvData);

	return PVRSRV_OK;
}
#endif


/* ***************************************************************************
 * Server-side bridge entry points
 */
 
static IMG_INT
PVRSRVBridgeSyncFbTimelineCreatePVR(IMG_UINT32 ui32DispatchTableEntry,
					  PVRSRV_BRIDGE_IN_SYNCFBTIMELINECREATEPVR *psSyncFbTimelineCreatePVRIN,
					  PVRSRV_BRIDGE_OUT_SYNCFBTIMELINECREATEPVR *psSyncFbTimelineCreatePVROUT,
					 CONNECTION_DATA *psConnection)
{
	IMG_CHAR *uiTimelineNameInt = NULL;
	PVRSRV_TIMELINE_SERVER * psTimelineInt = NULL;

	IMG_UINT32 ui32NextOffset = 0;
	IMG_BYTE   *pArrayArgsBuffer = NULL;
#if !defined(INTEGRITY_OS)
	IMG_BOOL bHaveEnoughSpace = IMG_FALSE;
#endif

	IMG_UINT32 ui32BufferSize = 
			(psSyncFbTimelineCreatePVRIN->ui32TimelineNameSize * sizeof(IMG_CHAR)) +
			0;





	if (ui32BufferSize != 0)
	{
#if !defined(INTEGRITY_OS)
		/* Try to use remainder of input buffer for copies if possible, word-aligned for safety. */
		IMG_UINT32 ui32InBufferOffset = PVR_ALIGN(sizeof(*psSyncFbTimelineCreatePVRIN), sizeof(unsigned long));
		IMG_UINT32 ui32InBufferExcessSize = ui32InBufferOffset >= PVRSRV_MAX_BRIDGE_IN_SIZE ? 0 :
			PVRSRV_MAX_BRIDGE_IN_SIZE - ui32InBufferOffset;

		bHaveEnoughSpace = ui32BufferSize <= ui32InBufferExcessSize;
		if (bHaveEnoughSpace)
		{
			IMG_BYTE *pInputBuffer = (IMG_BYTE *)psSyncFbTimelineCreatePVRIN;

			pArrayArgsBuffer = &pInputBuffer[ui32InBufferOffset];
		}
		else
#endif
		{
			pArrayArgsBuffer = OSAllocMemNoStats(ui32BufferSize);

			if(!pArrayArgsBuffer)
			{
				psSyncFbTimelineCreatePVROUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;
				goto SyncFbTimelineCreatePVR_exit;
			}
		}
	}

	if (psSyncFbTimelineCreatePVRIN->ui32TimelineNameSize != 0)
	{
		uiTimelineNameInt = (IMG_CHAR*)(((IMG_UINT8 *)pArrayArgsBuffer) + ui32NextOffset);
		ui32NextOffset += psSyncFbTimelineCreatePVRIN->ui32TimelineNameSize * sizeof(IMG_CHAR);
	}

			/* Copy the data over */
			if (psSyncFbTimelineCreatePVRIN->ui32TimelineNameSize * sizeof(IMG_CHAR) > 0)
			{
				if ( OSCopyFromUser(NULL, uiTimelineNameInt, (const void __user *) psSyncFbTimelineCreatePVRIN->puiTimelineName, psSyncFbTimelineCreatePVRIN->ui32TimelineNameSize * sizeof(IMG_CHAR)) != PVRSRV_OK )
				{
					psSyncFbTimelineCreatePVROUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

					goto SyncFbTimelineCreatePVR_exit;
				}
			}


	psSyncFbTimelineCreatePVROUT->eError =
		SyncFbTimelineCreatePVR(
					psSyncFbTimelineCreatePVRIN->ui32TimelineNameSize,
					uiTimelineNameInt,
					&psTimelineInt);
	/* Exit early if bridged call fails */
	if(psSyncFbTimelineCreatePVROUT->eError != PVRSRV_OK)
	{
		goto SyncFbTimelineCreatePVR_exit;
	}

	/* Lock over handle creation. */
	LockHandle();





	psSyncFbTimelineCreatePVROUT->eError = PVRSRVAllocHandleUnlocked(psConnection->psProcessHandleBase->psHandleBase,

							&psSyncFbTimelineCreatePVROUT->hTimeline,
							(void *) psTimelineInt,
							PVRSRV_HANDLE_TYPE_PVRSRV_TIMELINE_SERVER,
							PVRSRV_HANDLE_ALLOC_FLAG_NONE
							,(PFN_HANDLE_RELEASE)&SyncFbTimelineRelease);
	if (psSyncFbTimelineCreatePVROUT->eError != PVRSRV_OK)
	{
		UnlockHandle();
		goto SyncFbTimelineCreatePVR_exit;
	}

	/* Release now we have created handles. */
	UnlockHandle();



SyncFbTimelineCreatePVR_exit:



	if (psSyncFbTimelineCreatePVROUT->eError != PVRSRV_OK)
	{
		if (psTimelineInt)
		{
			SyncFbTimelineRelease(psTimelineInt);
		}
	}

	/* Allocated space should be equal to the last updated offset */
	PVR_ASSERT(ui32BufferSize == ui32NextOffset);

#if defined(INTEGRITY_OS)
	if(pArrayArgsBuffer)
#else
	if(!bHaveEnoughSpace && pArrayArgsBuffer)
#endif
		OSFreeMemNoStats(pArrayArgsBuffer);


	return 0;
}


static IMG_INT
PVRSRVBridgeSyncFbTimelineRelease(IMG_UINT32 ui32DispatchTableEntry,
					  PVRSRV_BRIDGE_IN_SYNCFBTIMELINERELEASE *psSyncFbTimelineReleaseIN,
					  PVRSRV_BRIDGE_OUT_SYNCFBTIMELINERELEASE *psSyncFbTimelineReleaseOUT,
					 CONNECTION_DATA *psConnection)
{









	/* Lock over handle destruction. */
	LockHandle();





	psSyncFbTimelineReleaseOUT->eError =
		PVRSRVReleaseHandleUnlocked(psConnection->psProcessHandleBase->psHandleBase,
					(IMG_HANDLE) psSyncFbTimelineReleaseIN->hTimeline,
					PVRSRV_HANDLE_TYPE_PVRSRV_TIMELINE_SERVER);
	if ((psSyncFbTimelineReleaseOUT->eError != PVRSRV_OK) &&
	    (psSyncFbTimelineReleaseOUT->eError != PVRSRV_ERROR_RETRY))
	{
		PVR_DPF((PVR_DBG_ERROR,
		        "PVRSRVBridgeSyncFbTimelineRelease: %s",
		        PVRSRVGetErrorStringKM(psSyncFbTimelineReleaseOUT->eError)));
		UnlockHandle();
		goto SyncFbTimelineRelease_exit;
	}

	/* Release now we have destroyed handles. */
	UnlockHandle();



SyncFbTimelineRelease_exit:




	return 0;
}


static IMG_INT
PVRSRVBridgeSyncFbFenceDup(IMG_UINT32 ui32DispatchTableEntry,
					  PVRSRV_BRIDGE_IN_SYNCFBFENCEDUP *psSyncFbFenceDupIN,
					  PVRSRV_BRIDGE_OUT_SYNCFBFENCEDUP *psSyncFbFenceDupOUT,
					 CONNECTION_DATA *psConnection)
{
	IMG_HANDLE hInFence = psSyncFbFenceDupIN->hInFence;
	PVRSRV_FENCE_SERVER * psInFenceInt = NULL;
	PVRSRV_FENCE_SERVER * psOutFenceInt = NULL;







	/* Lock over handle lookup. */
	LockHandle();





					/* Look up the address from the handle */
					psSyncFbFenceDupOUT->eError =
						PVRSRVLookupHandleUnlocked(psConnection->psProcessHandleBase->psHandleBase,
											(void **) &psInFenceInt,
											hInFence,
											PVRSRV_HANDLE_TYPE_PVRSRV_FENCE_SERVER,
											IMG_TRUE);
					if(psSyncFbFenceDupOUT->eError != PVRSRV_OK)
					{
						UnlockHandle();
						goto SyncFbFenceDup_exit;
					}
	/* Release now we have looked up handles. */
	UnlockHandle();

	psSyncFbFenceDupOUT->eError =
		SyncFbFenceDup(
					psInFenceInt,
					&psOutFenceInt);
	/* Exit early if bridged call fails */
	if(psSyncFbFenceDupOUT->eError != PVRSRV_OK)
	{
		goto SyncFbFenceDup_exit;
	}

	/* Lock over handle creation. */
	LockHandle();





	psSyncFbFenceDupOUT->eError = PVRSRVAllocHandleUnlocked(psConnection->psProcessHandleBase->psHandleBase,

							&psSyncFbFenceDupOUT->hOutFence,
							(void *) psOutFenceInt,
							PVRSRV_HANDLE_TYPE_PVRSRV_FENCE_SERVER,
							PVRSRV_HANDLE_ALLOC_FLAG_MULTI
							,(PFN_HANDLE_RELEASE)&SyncFbFenceRelease);
	if (psSyncFbFenceDupOUT->eError != PVRSRV_OK)
	{
		UnlockHandle();
		goto SyncFbFenceDup_exit;
	}

	/* Release now we have created handles. */
	UnlockHandle();



SyncFbFenceDup_exit:

	/* Lock over handle lookup cleanup. */
	LockHandle();







					/* Unreference the previously looked up handle */
					if(psInFenceInt)
					{
						PVRSRVReleaseHandleUnlocked(psConnection->psProcessHandleBase->psHandleBase,
										hInFence,
										PVRSRV_HANDLE_TYPE_PVRSRV_FENCE_SERVER);
					}
	/* Release now we have cleaned up look up handles. */
	UnlockHandle();

	if (psSyncFbFenceDupOUT->eError != PVRSRV_OK)
	{
		if (psOutFenceInt)
		{
			SyncFbFenceRelease(psOutFenceInt);
		}
	}


	return 0;
}


static IMG_INT
PVRSRVBridgeSyncFbFenceMerge(IMG_UINT32 ui32DispatchTableEntry,
					  PVRSRV_BRIDGE_IN_SYNCFBFENCEMERGE *psSyncFbFenceMergeIN,
					  PVRSRV_BRIDGE_OUT_SYNCFBFENCEMERGE *psSyncFbFenceMergeOUT,
					 CONNECTION_DATA *psConnection)
{
	IMG_HANDLE hInFence1 = psSyncFbFenceMergeIN->hInFence1;
	PVRSRV_FENCE_SERVER * psInFence1Int = NULL;
	IMG_HANDLE hInFence2 = psSyncFbFenceMergeIN->hInFence2;
	PVRSRV_FENCE_SERVER * psInFence2Int = NULL;
	IMG_CHAR *uiFenceNameInt = NULL;
	PVRSRV_FENCE_SERVER * psOutFenceInt = NULL;

	IMG_UINT32 ui32NextOffset = 0;
	IMG_BYTE   *pArrayArgsBuffer = NULL;
#if !defined(INTEGRITY_OS)
	IMG_BOOL bHaveEnoughSpace = IMG_FALSE;
#endif

	IMG_UINT32 ui32BufferSize = 
			(psSyncFbFenceMergeIN->ui32FenceNameSize * sizeof(IMG_CHAR)) +
			0;





	if (ui32BufferSize != 0)
	{
#if !defined(INTEGRITY_OS)
		/* Try to use remainder of input buffer for copies if possible, word-aligned for safety. */
		IMG_UINT32 ui32InBufferOffset = PVR_ALIGN(sizeof(*psSyncFbFenceMergeIN), sizeof(unsigned long));
		IMG_UINT32 ui32InBufferExcessSize = ui32InBufferOffset >= PVRSRV_MAX_BRIDGE_IN_SIZE ? 0 :
			PVRSRV_MAX_BRIDGE_IN_SIZE - ui32InBufferOffset;

		bHaveEnoughSpace = ui32BufferSize <= ui32InBufferExcessSize;
		if (bHaveEnoughSpace)
		{
			IMG_BYTE *pInputBuffer = (IMG_BYTE *)psSyncFbFenceMergeIN;

			pArrayArgsBuffer = &pInputBuffer[ui32InBufferOffset];
		}
		else
#endif
		{
			pArrayArgsBuffer = OSAllocMemNoStats(ui32BufferSize);

			if(!pArrayArgsBuffer)
			{
				psSyncFbFenceMergeOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;
				goto SyncFbFenceMerge_exit;
			}
		}
	}

	if (psSyncFbFenceMergeIN->ui32FenceNameSize != 0)
	{
		uiFenceNameInt = (IMG_CHAR*)(((IMG_UINT8 *)pArrayArgsBuffer) + ui32NextOffset);
		ui32NextOffset += psSyncFbFenceMergeIN->ui32FenceNameSize * sizeof(IMG_CHAR);
	}

			/* Copy the data over */
			if (psSyncFbFenceMergeIN->ui32FenceNameSize * sizeof(IMG_CHAR) > 0)
			{
				if ( OSCopyFromUser(NULL, uiFenceNameInt, (const void __user *) psSyncFbFenceMergeIN->puiFenceName, psSyncFbFenceMergeIN->ui32FenceNameSize * sizeof(IMG_CHAR)) != PVRSRV_OK )
				{
					psSyncFbFenceMergeOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

					goto SyncFbFenceMerge_exit;
				}
			}

	/* Lock over handle lookup. */
	LockHandle();





					/* Look up the address from the handle */
					psSyncFbFenceMergeOUT->eError =
						PVRSRVLookupHandleUnlocked(psConnection->psProcessHandleBase->psHandleBase,
											(void **) &psInFence1Int,
											hInFence1,
											PVRSRV_HANDLE_TYPE_PVRSRV_FENCE_SERVER,
											IMG_TRUE);
					if(psSyncFbFenceMergeOUT->eError != PVRSRV_OK)
					{
						UnlockHandle();
						goto SyncFbFenceMerge_exit;
					}





					/* Look up the address from the handle */
					psSyncFbFenceMergeOUT->eError =
						PVRSRVLookupHandleUnlocked(psConnection->psProcessHandleBase->psHandleBase,
											(void **) &psInFence2Int,
											hInFence2,
											PVRSRV_HANDLE_TYPE_PVRSRV_FENCE_SERVER,
											IMG_TRUE);
					if(psSyncFbFenceMergeOUT->eError != PVRSRV_OK)
					{
						UnlockHandle();
						goto SyncFbFenceMerge_exit;
					}
	/* Release now we have looked up handles. */
	UnlockHandle();

	psSyncFbFenceMergeOUT->eError =
		SyncFbFenceMerge(
					psInFence1Int,
					psInFence2Int,
					psSyncFbFenceMergeIN->ui32FenceNameSize,
					uiFenceNameInt,
					&psOutFenceInt);
	/* Exit early if bridged call fails */
	if(psSyncFbFenceMergeOUT->eError != PVRSRV_OK)
	{
		goto SyncFbFenceMerge_exit;
	}

	/* Lock over handle creation. */
	LockHandle();





	psSyncFbFenceMergeOUT->eError = PVRSRVAllocHandleUnlocked(psConnection->psProcessHandleBase->psHandleBase,

							&psSyncFbFenceMergeOUT->hOutFence,
							(void *) psOutFenceInt,
							PVRSRV_HANDLE_TYPE_PVRSRV_FENCE_SERVER,
							PVRSRV_HANDLE_ALLOC_FLAG_MULTI
							,(PFN_HANDLE_RELEASE)&SyncFbFenceRelease);
	if (psSyncFbFenceMergeOUT->eError != PVRSRV_OK)
	{
		UnlockHandle();
		goto SyncFbFenceMerge_exit;
	}

	/* Release now we have created handles. */
	UnlockHandle();



SyncFbFenceMerge_exit:

	/* Lock over handle lookup cleanup. */
	LockHandle();







					/* Unreference the previously looked up handle */
					if(psInFence1Int)
					{
						PVRSRVReleaseHandleUnlocked(psConnection->psProcessHandleBase->psHandleBase,
										hInFence1,
										PVRSRV_HANDLE_TYPE_PVRSRV_FENCE_SERVER);
					}






					/* Unreference the previously looked up handle */
					if(psInFence2Int)
					{
						PVRSRVReleaseHandleUnlocked(psConnection->psProcessHandleBase->psHandleBase,
										hInFence2,
										PVRSRV_HANDLE_TYPE_PVRSRV_FENCE_SERVER);
					}
	/* Release now we have cleaned up look up handles. */
	UnlockHandle();

	if (psSyncFbFenceMergeOUT->eError != PVRSRV_OK)
	{
		if (psOutFenceInt)
		{
			SyncFbFenceRelease(psOutFenceInt);
		}
	}

	/* Allocated space should be equal to the last updated offset */
	PVR_ASSERT(ui32BufferSize == ui32NextOffset);

#if defined(INTEGRITY_OS)
	if(pArrayArgsBuffer)
#else
	if(!bHaveEnoughSpace && pArrayArgsBuffer)
#endif
		OSFreeMemNoStats(pArrayArgsBuffer);


	return 0;
}


static IMG_INT
PVRSRVBridgeSyncFbFenceRelease(IMG_UINT32 ui32DispatchTableEntry,
					  PVRSRV_BRIDGE_IN_SYNCFBFENCERELEASE *psSyncFbFenceReleaseIN,
					  PVRSRV_BRIDGE_OUT_SYNCFBFENCERELEASE *psSyncFbFenceReleaseOUT,
					 CONNECTION_DATA *psConnection)
{









	/* Lock over handle destruction. */
	LockHandle();





	psSyncFbFenceReleaseOUT->eError =
		PVRSRVReleaseHandleUnlocked(psConnection->psProcessHandleBase->psHandleBase,
					(IMG_HANDLE) psSyncFbFenceReleaseIN->hFence,
					PVRSRV_HANDLE_TYPE_PVRSRV_FENCE_SERVER);
	if ((psSyncFbFenceReleaseOUT->eError != PVRSRV_OK) &&
	    (psSyncFbFenceReleaseOUT->eError != PVRSRV_ERROR_RETRY))
	{
		PVR_DPF((PVR_DBG_ERROR,
		        "PVRSRVBridgeSyncFbFenceRelease: %s",
		        PVRSRVGetErrorStringKM(psSyncFbFenceReleaseOUT->eError)));
		UnlockHandle();
		goto SyncFbFenceRelease_exit;
	}

	/* Release now we have destroyed handles. */
	UnlockHandle();



SyncFbFenceRelease_exit:




	return 0;
}


static IMG_INT
PVRSRVBridgeSyncFbFenceWait(IMG_UINT32 ui32DispatchTableEntry,
					  PVRSRV_BRIDGE_IN_SYNCFBFENCEWAIT *psSyncFbFenceWaitIN,
					  PVRSRV_BRIDGE_OUT_SYNCFBFENCEWAIT *psSyncFbFenceWaitOUT,
					 CONNECTION_DATA *psConnection)
{
	IMG_HANDLE hFence = psSyncFbFenceWaitIN->hFence;
	PVRSRV_FENCE_SERVER * psFenceInt = NULL;







	/* Lock over handle lookup. */
	LockHandle();





					/* Look up the address from the handle */
					psSyncFbFenceWaitOUT->eError =
						PVRSRVLookupHandleUnlocked(psConnection->psProcessHandleBase->psHandleBase,
											(void **) &psFenceInt,
											hFence,
											PVRSRV_HANDLE_TYPE_PVRSRV_FENCE_SERVER,
											IMG_TRUE);
					if(psSyncFbFenceWaitOUT->eError != PVRSRV_OK)
					{
						UnlockHandle();
						goto SyncFbFenceWait_exit;
					}
	/* Release now we have looked up handles. */
	UnlockHandle();

	psSyncFbFenceWaitOUT->eError =
		SyncFbFenceWait(
					psFenceInt,
					psSyncFbFenceWaitIN->ui32Timeout);




SyncFbFenceWait_exit:

	/* Lock over handle lookup cleanup. */
	LockHandle();







					/* Unreference the previously looked up handle */
					if(psFenceInt)
					{
						PVRSRVReleaseHandleUnlocked(psConnection->psProcessHandleBase->psHandleBase,
										hFence,
										PVRSRV_HANDLE_TYPE_PVRSRV_FENCE_SERVER);
					}
	/* Release now we have cleaned up look up handles. */
	UnlockHandle();


	return 0;
}


static IMG_INT
PVRSRVBridgeSyncFbFenceDump(IMG_UINT32 ui32DispatchTableEntry,
					  PVRSRV_BRIDGE_IN_SYNCFBFENCEDUMP *psSyncFbFenceDumpIN,
					  PVRSRV_BRIDGE_OUT_SYNCFBFENCEDUMP *psSyncFbFenceDumpOUT,
					 CONNECTION_DATA *psConnection)
{
	IMG_HANDLE hFence = psSyncFbFenceDumpIN->hFence;
	PVRSRV_FENCE_SERVER * psFenceInt = NULL;
	IMG_CHAR *uiFileStrInt = NULL;

	IMG_UINT32 ui32NextOffset = 0;
	IMG_BYTE   *pArrayArgsBuffer = NULL;
#if !defined(INTEGRITY_OS)
	IMG_BOOL bHaveEnoughSpace = IMG_FALSE;
#endif

	IMG_UINT32 ui32BufferSize = 
			(psSyncFbFenceDumpIN->ui32FileStrLength * sizeof(IMG_CHAR)) +
			0;





	if (ui32BufferSize != 0)
	{
#if !defined(INTEGRITY_OS)
		/* Try to use remainder of input buffer for copies if possible, word-aligned for safety. */
		IMG_UINT32 ui32InBufferOffset = PVR_ALIGN(sizeof(*psSyncFbFenceDumpIN), sizeof(unsigned long));
		IMG_UINT32 ui32InBufferExcessSize = ui32InBufferOffset >= PVRSRV_MAX_BRIDGE_IN_SIZE ? 0 :
			PVRSRV_MAX_BRIDGE_IN_SIZE - ui32InBufferOffset;

		bHaveEnoughSpace = ui32BufferSize <= ui32InBufferExcessSize;
		if (bHaveEnoughSpace)
		{
			IMG_BYTE *pInputBuffer = (IMG_BYTE *)psSyncFbFenceDumpIN;

			pArrayArgsBuffer = &pInputBuffer[ui32InBufferOffset];
		}
		else
#endif
		{
			pArrayArgsBuffer = OSAllocMemNoStats(ui32BufferSize);

			if(!pArrayArgsBuffer)
			{
				psSyncFbFenceDumpOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;
				goto SyncFbFenceDump_exit;
			}
		}
	}

	if (psSyncFbFenceDumpIN->ui32FileStrLength != 0)
	{
		uiFileStrInt = (IMG_CHAR*)(((IMG_UINT8 *)pArrayArgsBuffer) + ui32NextOffset);
		ui32NextOffset += psSyncFbFenceDumpIN->ui32FileStrLength * sizeof(IMG_CHAR);
	}

			/* Copy the data over */
			if (psSyncFbFenceDumpIN->ui32FileStrLength * sizeof(IMG_CHAR) > 0)
			{
				if ( OSCopyFromUser(NULL, uiFileStrInt, (const void __user *) psSyncFbFenceDumpIN->puiFileStr, psSyncFbFenceDumpIN->ui32FileStrLength * sizeof(IMG_CHAR)) != PVRSRV_OK )
				{
					psSyncFbFenceDumpOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

					goto SyncFbFenceDump_exit;
				}
			}

	/* Lock over handle lookup. */
	LockHandle();





					/* Look up the address from the handle */
					psSyncFbFenceDumpOUT->eError =
						PVRSRVLookupHandleUnlocked(psConnection->psProcessHandleBase->psHandleBase,
											(void **) &psFenceInt,
											hFence,
											PVRSRV_HANDLE_TYPE_PVRSRV_FENCE_SERVER,
											IMG_TRUE);
					if(psSyncFbFenceDumpOUT->eError != PVRSRV_OK)
					{
						UnlockHandle();
						goto SyncFbFenceDump_exit;
					}
	/* Release now we have looked up handles. */
	UnlockHandle();

	psSyncFbFenceDumpOUT->eError =
		SyncFbFenceDump(
					psFenceInt,
					psSyncFbFenceDumpIN->ui32Line,
					psSyncFbFenceDumpIN->ui32FileStrLength,
					uiFileStrInt);




SyncFbFenceDump_exit:

	/* Lock over handle lookup cleanup. */
	LockHandle();







					/* Unreference the previously looked up handle */
					if(psFenceInt)
					{
						PVRSRVReleaseHandleUnlocked(psConnection->psProcessHandleBase->psHandleBase,
										hFence,
										PVRSRV_HANDLE_TYPE_PVRSRV_FENCE_SERVER);
					}
	/* Release now we have cleaned up look up handles. */
	UnlockHandle();

	/* Allocated space should be equal to the last updated offset */
	PVR_ASSERT(ui32BufferSize == ui32NextOffset);

#if defined(INTEGRITY_OS)
	if(pArrayArgsBuffer)
#else
	if(!bHaveEnoughSpace && pArrayArgsBuffer)
#endif
		OSFreeMemNoStats(pArrayArgsBuffer);


	return 0;
}


static IMG_INT
PVRSRVBridgeSyncFbTimelineCreateSW(IMG_UINT32 ui32DispatchTableEntry,
					  PVRSRV_BRIDGE_IN_SYNCFBTIMELINECREATESW *psSyncFbTimelineCreateSWIN,
					  PVRSRV_BRIDGE_OUT_SYNCFBTIMELINECREATESW *psSyncFbTimelineCreateSWOUT,
					 CONNECTION_DATA *psConnection)
{
	IMG_CHAR *uiTimelineNameInt = NULL;
	PVRSRV_TIMELINE_SERVER * psTimelineInt = NULL;

	IMG_UINT32 ui32NextOffset = 0;
	IMG_BYTE   *pArrayArgsBuffer = NULL;
#if !defined(INTEGRITY_OS)
	IMG_BOOL bHaveEnoughSpace = IMG_FALSE;
#endif

	IMG_UINT32 ui32BufferSize = 
			(psSyncFbTimelineCreateSWIN->ui32TimelineNameSize * sizeof(IMG_CHAR)) +
			0;





	if (ui32BufferSize != 0)
	{
#if !defined(INTEGRITY_OS)
		/* Try to use remainder of input buffer for copies if possible, word-aligned for safety. */
		IMG_UINT32 ui32InBufferOffset = PVR_ALIGN(sizeof(*psSyncFbTimelineCreateSWIN), sizeof(unsigned long));
		IMG_UINT32 ui32InBufferExcessSize = ui32InBufferOffset >= PVRSRV_MAX_BRIDGE_IN_SIZE ? 0 :
			PVRSRV_MAX_BRIDGE_IN_SIZE - ui32InBufferOffset;

		bHaveEnoughSpace = ui32BufferSize <= ui32InBufferExcessSize;
		if (bHaveEnoughSpace)
		{
			IMG_BYTE *pInputBuffer = (IMG_BYTE *)psSyncFbTimelineCreateSWIN;

			pArrayArgsBuffer = &pInputBuffer[ui32InBufferOffset];
		}
		else
#endif
		{
			pArrayArgsBuffer = OSAllocMemNoStats(ui32BufferSize);

			if(!pArrayArgsBuffer)
			{
				psSyncFbTimelineCreateSWOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;
				goto SyncFbTimelineCreateSW_exit;
			}
		}
	}

	if (psSyncFbTimelineCreateSWIN->ui32TimelineNameSize != 0)
	{
		uiTimelineNameInt = (IMG_CHAR*)(((IMG_UINT8 *)pArrayArgsBuffer) + ui32NextOffset);
		ui32NextOffset += psSyncFbTimelineCreateSWIN->ui32TimelineNameSize * sizeof(IMG_CHAR);
	}

			/* Copy the data over */
			if (psSyncFbTimelineCreateSWIN->ui32TimelineNameSize * sizeof(IMG_CHAR) > 0)
			{
				if ( OSCopyFromUser(NULL, uiTimelineNameInt, (const void __user *) psSyncFbTimelineCreateSWIN->puiTimelineName, psSyncFbTimelineCreateSWIN->ui32TimelineNameSize * sizeof(IMG_CHAR)) != PVRSRV_OK )
				{
					psSyncFbTimelineCreateSWOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

					goto SyncFbTimelineCreateSW_exit;
				}
			}


	psSyncFbTimelineCreateSWOUT->eError =
		SyncFbTimelineCreateSW(
					psSyncFbTimelineCreateSWIN->ui32TimelineNameSize,
					uiTimelineNameInt,
					&psTimelineInt);
	/* Exit early if bridged call fails */
	if(psSyncFbTimelineCreateSWOUT->eError != PVRSRV_OK)
	{
		goto SyncFbTimelineCreateSW_exit;
	}

	/* Lock over handle creation. */
	LockHandle();





	psSyncFbTimelineCreateSWOUT->eError = PVRSRVAllocHandleUnlocked(psConnection->psProcessHandleBase->psHandleBase,

							&psSyncFbTimelineCreateSWOUT->hTimeline,
							(void *) psTimelineInt,
							PVRSRV_HANDLE_TYPE_PVRSRV_TIMELINE_SERVER,
							PVRSRV_HANDLE_ALLOC_FLAG_NONE
							,(PFN_HANDLE_RELEASE)&SyncFbTimelineRelease);
	if (psSyncFbTimelineCreateSWOUT->eError != PVRSRV_OK)
	{
		UnlockHandle();
		goto SyncFbTimelineCreateSW_exit;
	}

	/* Release now we have created handles. */
	UnlockHandle();



SyncFbTimelineCreateSW_exit:



	if (psSyncFbTimelineCreateSWOUT->eError != PVRSRV_OK)
	{
		if (psTimelineInt)
		{
			SyncFbTimelineRelease(psTimelineInt);
		}
	}

	/* Allocated space should be equal to the last updated offset */
	PVR_ASSERT(ui32BufferSize == ui32NextOffset);

#if defined(INTEGRITY_OS)
	if(pArrayArgsBuffer)
#else
	if(!bHaveEnoughSpace && pArrayArgsBuffer)
#endif
		OSFreeMemNoStats(pArrayArgsBuffer);


	return 0;
}


static IMG_INT
PVRSRVBridgeSyncFbFenceCreateSW(IMG_UINT32 ui32DispatchTableEntry,
					  PVRSRV_BRIDGE_IN_SYNCFBFENCECREATESW *psSyncFbFenceCreateSWIN,
					  PVRSRV_BRIDGE_OUT_SYNCFBFENCECREATESW *psSyncFbFenceCreateSWOUT,
					 CONNECTION_DATA *psConnection)
{
	IMG_HANDLE hTimeline = psSyncFbFenceCreateSWIN->hTimeline;
	PVRSRV_TIMELINE_SERVER * psTimelineInt = NULL;
	IMG_CHAR *uiFenceNameInt = NULL;
	PVRSRV_FENCE_SERVER * psOutFenceInt = NULL;

	IMG_UINT32 ui32NextOffset = 0;
	IMG_BYTE   *pArrayArgsBuffer = NULL;
#if !defined(INTEGRITY_OS)
	IMG_BOOL bHaveEnoughSpace = IMG_FALSE;
#endif

	IMG_UINT32 ui32BufferSize = 
			(psSyncFbFenceCreateSWIN->ui32FenceNameSize * sizeof(IMG_CHAR)) +
			0;





	if (ui32BufferSize != 0)
	{
#if !defined(INTEGRITY_OS)
		/* Try to use remainder of input buffer for copies if possible, word-aligned for safety. */
		IMG_UINT32 ui32InBufferOffset = PVR_ALIGN(sizeof(*psSyncFbFenceCreateSWIN), sizeof(unsigned long));
		IMG_UINT32 ui32InBufferExcessSize = ui32InBufferOffset >= PVRSRV_MAX_BRIDGE_IN_SIZE ? 0 :
			PVRSRV_MAX_BRIDGE_IN_SIZE - ui32InBufferOffset;

		bHaveEnoughSpace = ui32BufferSize <= ui32InBufferExcessSize;
		if (bHaveEnoughSpace)
		{
			IMG_BYTE *pInputBuffer = (IMG_BYTE *)psSyncFbFenceCreateSWIN;

			pArrayArgsBuffer = &pInputBuffer[ui32InBufferOffset];
		}
		else
#endif
		{
			pArrayArgsBuffer = OSAllocMemNoStats(ui32BufferSize);

			if(!pArrayArgsBuffer)
			{
				psSyncFbFenceCreateSWOUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;
				goto SyncFbFenceCreateSW_exit;
			}
		}
	}

	if (psSyncFbFenceCreateSWIN->ui32FenceNameSize != 0)
	{
		uiFenceNameInt = (IMG_CHAR*)(((IMG_UINT8 *)pArrayArgsBuffer) + ui32NextOffset);
		ui32NextOffset += psSyncFbFenceCreateSWIN->ui32FenceNameSize * sizeof(IMG_CHAR);
	}

			/* Copy the data over */
			if (psSyncFbFenceCreateSWIN->ui32FenceNameSize * sizeof(IMG_CHAR) > 0)
			{
				if ( OSCopyFromUser(NULL, uiFenceNameInt, (const void __user *) psSyncFbFenceCreateSWIN->puiFenceName, psSyncFbFenceCreateSWIN->ui32FenceNameSize * sizeof(IMG_CHAR)) != PVRSRV_OK )
				{
					psSyncFbFenceCreateSWOUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

					goto SyncFbFenceCreateSW_exit;
				}
			}

	/* Lock over handle lookup. */
	LockHandle();





					/* Look up the address from the handle */
					psSyncFbFenceCreateSWOUT->eError =
						PVRSRVLookupHandleUnlocked(psConnection->psProcessHandleBase->psHandleBase,
											(void **) &psTimelineInt,
											hTimeline,
											PVRSRV_HANDLE_TYPE_PVRSRV_TIMELINE_SERVER,
											IMG_TRUE);
					if(psSyncFbFenceCreateSWOUT->eError != PVRSRV_OK)
					{
						UnlockHandle();
						goto SyncFbFenceCreateSW_exit;
					}
	/* Release now we have looked up handles. */
	UnlockHandle();

	psSyncFbFenceCreateSWOUT->eError =
		SyncFbFenceCreateSW(
					psTimelineInt,
					psSyncFbFenceCreateSWIN->ui32FenceNameSize,
					uiFenceNameInt,
					&psOutFenceInt);
	/* Exit early if bridged call fails */
	if(psSyncFbFenceCreateSWOUT->eError != PVRSRV_OK)
	{
		goto SyncFbFenceCreateSW_exit;
	}

	/* Lock over handle creation. */
	LockHandle();





	psSyncFbFenceCreateSWOUT->eError = PVRSRVAllocHandleUnlocked(psConnection->psProcessHandleBase->psHandleBase,

							&psSyncFbFenceCreateSWOUT->hOutFence,
							(void *) psOutFenceInt,
							PVRSRV_HANDLE_TYPE_PVRSRV_FENCE_SERVER,
							PVRSRV_HANDLE_ALLOC_FLAG_MULTI
							,(PFN_HANDLE_RELEASE)&SyncFbFenceRelease);
	if (psSyncFbFenceCreateSWOUT->eError != PVRSRV_OK)
	{
		UnlockHandle();
		goto SyncFbFenceCreateSW_exit;
	}

	/* Release now we have created handles. */
	UnlockHandle();



SyncFbFenceCreateSW_exit:

	/* Lock over handle lookup cleanup. */
	LockHandle();







					/* Unreference the previously looked up handle */
					if(psTimelineInt)
					{
						PVRSRVReleaseHandleUnlocked(psConnection->psProcessHandleBase->psHandleBase,
										hTimeline,
										PVRSRV_HANDLE_TYPE_PVRSRV_TIMELINE_SERVER);
					}
	/* Release now we have cleaned up look up handles. */
	UnlockHandle();

	if (psSyncFbFenceCreateSWOUT->eError != PVRSRV_OK)
	{
		if (psOutFenceInt)
		{
			SyncFbFenceRelease(psOutFenceInt);
		}
	}

	/* Allocated space should be equal to the last updated offset */
	PVR_ASSERT(ui32BufferSize == ui32NextOffset);

#if defined(INTEGRITY_OS)
	if(pArrayArgsBuffer)
#else
	if(!bHaveEnoughSpace && pArrayArgsBuffer)
#endif
		OSFreeMemNoStats(pArrayArgsBuffer);


	return 0;
}


static IMG_INT
PVRSRVBridgeSyncFbTimelineAdvanceSW(IMG_UINT32 ui32DispatchTableEntry,
					  PVRSRV_BRIDGE_IN_SYNCFBTIMELINEADVANCESW *psSyncFbTimelineAdvanceSWIN,
					  PVRSRV_BRIDGE_OUT_SYNCFBTIMELINEADVANCESW *psSyncFbTimelineAdvanceSWOUT,
					 CONNECTION_DATA *psConnection)
{
	IMG_HANDLE hTimeline = psSyncFbTimelineAdvanceSWIN->hTimeline;
	PVRSRV_TIMELINE_SERVER * psTimelineInt = NULL;







	/* Lock over handle lookup. */
	LockHandle();





					/* Look up the address from the handle */
					psSyncFbTimelineAdvanceSWOUT->eError =
						PVRSRVLookupHandleUnlocked(psConnection->psProcessHandleBase->psHandleBase,
											(void **) &psTimelineInt,
											hTimeline,
											PVRSRV_HANDLE_TYPE_PVRSRV_TIMELINE_SERVER,
											IMG_TRUE);
					if(psSyncFbTimelineAdvanceSWOUT->eError != PVRSRV_OK)
					{
						UnlockHandle();
						goto SyncFbTimelineAdvanceSW_exit;
					}
	/* Release now we have looked up handles. */
	UnlockHandle();

	psSyncFbTimelineAdvanceSWOUT->eError =
		SyncFbTimelineAdvanceSW(
					psTimelineInt);




SyncFbTimelineAdvanceSW_exit:

	/* Lock over handle lookup cleanup. */
	LockHandle();







					/* Unreference the previously looked up handle */
					if(psTimelineInt)
					{
						PVRSRVReleaseHandleUnlocked(psConnection->psProcessHandleBase->psHandleBase,
										hTimeline,
										PVRSRV_HANDLE_TYPE_PVRSRV_TIMELINE_SERVER);
					}
	/* Release now we have cleaned up look up handles. */
	UnlockHandle();


	return 0;
}


#if defined(SUPPORT_INSECURE_EXPORT)
static IMG_INT
PVRSRVBridgeSyncFbFenceExportInsecure(IMG_UINT32 ui32DispatchTableEntry,
					  PVRSRV_BRIDGE_IN_SYNCFBFENCEEXPORTINSECURE *psSyncFbFenceExportInsecureIN,
					  PVRSRV_BRIDGE_OUT_SYNCFBFENCEEXPORTINSECURE *psSyncFbFenceExportInsecureOUT,
					 CONNECTION_DATA *psConnection)
{
	IMG_HANDLE hFence = psSyncFbFenceExportInsecureIN->hFence;
	PVRSRV_FENCE_SERVER * psFenceInt = NULL;
	PVRSRV_FENCE_EXPORT * psExportInt = NULL;
	IMG_HANDLE hExportInt = NULL;







	/* Lock over handle lookup. */
	LockHandle();





					/* Look up the address from the handle */
					psSyncFbFenceExportInsecureOUT->eError =
						PVRSRVLookupHandleUnlocked(psConnection->psProcessHandleBase->psHandleBase,
											(void **) &psFenceInt,
											hFence,
											PVRSRV_HANDLE_TYPE_PVRSRV_FENCE_SERVER,
											IMG_TRUE);
					if(psSyncFbFenceExportInsecureOUT->eError != PVRSRV_OK)
					{
						UnlockHandle();
						goto SyncFbFenceExportInsecure_exit;
					}
	/* Release now we have looked up handles. */
	UnlockHandle();

	psSyncFbFenceExportInsecureOUT->eError =
		SyncFbFenceExportInsecure(
					psFenceInt,
					&psExportInt);
	/* Exit early if bridged call fails */
	if(psSyncFbFenceExportInsecureOUT->eError != PVRSRV_OK)
	{
		goto SyncFbFenceExportInsecure_exit;
	}

	/* Lock over handle creation. */
	LockHandle();

	/*
	 * For cases where we need a cross process handle we actually allocate two.
	 * 
	 * The first one is a connection specific handle and it gets given the real
	 * release function. This handle does *NOT* get returned to the caller. It's
	 * purpose is to release any leaked resources when we either have a bad or
	 * abnormally terminated client. If we didn't do this then the resource
	 * wouldn't be freed until driver unload. If the resource is freed normally,
	 * this handle can be looked up via the cross process handle and then
	 * released accordingly.
	 * 
	 * The second one is a cross process handle and it gets given a noop release
	 * function. This handle does get returned to the caller.
	 */




	psSyncFbFenceExportInsecureOUT->eError = PVRSRVAllocHandleUnlocked(psConnection->psProcessHandleBase->psHandleBase,

							&hExportInt,
							(void *) psExportInt,
							PVRSRV_HANDLE_TYPE_PVRSRV_FENCE_EXPORT,
							PVRSRV_HANDLE_ALLOC_FLAG_NONE
							,(PFN_HANDLE_RELEASE)&SyncFbFenceExportDestroyInsecure);
	if (psSyncFbFenceExportInsecureOUT->eError != PVRSRV_OK)
	{
		UnlockHandle();
		goto SyncFbFenceExportInsecure_exit;
	}

	psSyncFbFenceExportInsecureOUT->eError = PVRSRVAllocHandleUnlocked(KERNEL_HANDLE_BASE,
							&psSyncFbFenceExportInsecureOUT->hExport,
							(void *) psExportInt,
							PVRSRV_HANDLE_TYPE_PVRSRV_FENCE_EXPORT,
							PVRSRV_HANDLE_ALLOC_FLAG_NONE,
							(PFN_HANDLE_RELEASE)&ReleaseExport);
	if (psSyncFbFenceExportInsecureOUT->eError != PVRSRV_OK)
	{
		UnlockHandle();
		goto SyncFbFenceExportInsecure_exit;
	}
	/* Release now we have created handles. */
	UnlockHandle();



SyncFbFenceExportInsecure_exit:

	/* Lock over handle lookup cleanup. */
	LockHandle();







					/* Unreference the previously looked up handle */
					if(psFenceInt)
					{
						PVRSRVReleaseHandleUnlocked(psConnection->psProcessHandleBase->psHandleBase,
										hFence,
										PVRSRV_HANDLE_TYPE_PVRSRV_FENCE_SERVER);
					}
	/* Release now we have cleaned up look up handles. */
	UnlockHandle();

	if (psSyncFbFenceExportInsecureOUT->eError != PVRSRV_OK)
	{
		/* Lock over handle creation cleanup. */
		LockHandle();
		if (psSyncFbFenceExportInsecureOUT->hExport)
		{


			PVRSRV_ERROR eError = PVRSRVReleaseHandleUnlocked(KERNEL_HANDLE_BASE,
						(IMG_HANDLE) psSyncFbFenceExportInsecureOUT->hExport,
						PVRSRV_HANDLE_TYPE_PVRSRV_FENCE_EXPORT);
			if ((eError != PVRSRV_OK) && (eError != PVRSRV_ERROR_RETRY))
			{
				PVR_DPF((PVR_DBG_ERROR,
				        "PVRSRVBridgeSyncFbFenceExportInsecure: %s",
				        PVRSRVGetErrorStringKM(eError)));
			}
			/* Releasing the handle should free/destroy/release the resource.
			 * This should never fail... */
			PVR_ASSERT((eError == PVRSRV_OK) || (eError == PVRSRV_ERROR_RETRY));

		}

		if (hExportInt)
		{
			PVRSRV_ERROR eError = PVRSRVReleaseHandleUnlocked(psConnection->psProcessHandleBase->psHandleBase,
						hExportInt,
						PVRSRV_HANDLE_TYPE_PVRSRV_FENCE_EXPORT);
			if ((eError != PVRSRV_OK) && (eError != PVRSRV_ERROR_RETRY))
			{
				PVR_DPF((PVR_DBG_ERROR,
				        "PVRSRVBridgeSyncFbFenceExportInsecure: %s",
				        PVRSRVGetErrorStringKM(eError)));
			}
			/* Releasing the handle should free/destroy/release the resource.
			 * This should never fail... */
			PVR_ASSERT((eError == PVRSRV_OK) || (eError == PVRSRV_ERROR_RETRY));

			/* Avoid freeing/destroying/releasing the resource a second time below */
			psExportInt = NULL;
		}

		/* Release now we have cleaned up creation handles. */
		UnlockHandle();
		if (psExportInt)
		{
			SyncFbFenceExportDestroyInsecure(psExportInt);
		}
	}


	return 0;
}

#else
#define PVRSRVBridgeSyncFbFenceExportInsecure NULL
#endif

#if defined(SUPPORT_INSECURE_EXPORT)
static IMG_INT
PVRSRVBridgeSyncFbFenceExportDestroyInsecure(IMG_UINT32 ui32DispatchTableEntry,
					  PVRSRV_BRIDGE_IN_SYNCFBFENCEEXPORTDESTROYINSECURE *psSyncFbFenceExportDestroyInsecureIN,
					  PVRSRV_BRIDGE_OUT_SYNCFBFENCEEXPORTDESTROYINSECURE *psSyncFbFenceExportDestroyInsecureOUT,
					 CONNECTION_DATA *psConnection)
{
	PVRSRV_FENCE_EXPORT * psExportInt = NULL;
	IMG_HANDLE hExportInt = NULL;



	PVR_UNREFERENCED_PARAMETER(psConnection);






	/* Lock over handle destruction. */
	LockHandle();


	psSyncFbFenceExportDestroyInsecureOUT->eError =
		PVRSRVLookupHandleUnlocked(KERNEL_HANDLE_BASE,
					(void **) &psExportInt,
					(IMG_HANDLE) psSyncFbFenceExportDestroyInsecureIN->hExport,
					PVRSRV_HANDLE_TYPE_PVRSRV_FENCE_EXPORT,
					IMG_FALSE);
	if (psSyncFbFenceExportDestroyInsecureOUT->eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,
		        "PVRSRVBridgeSyncFbFenceExportDestroyInsecure: %s",
		        PVRSRVGetErrorStringKM(psSyncFbFenceExportDestroyInsecureOUT->eError)));
	}
	PVR_ASSERT(psSyncFbFenceExportDestroyInsecureOUT->eError == PVRSRV_OK);

	/*
	 * Find the connection specific handle that represents the same data
	 * as the cross process handle as releasing it will actually call the
	 * data's real release function (see the function where the cross
	 * process handle is allocated for more details).
	 */
	psSyncFbFenceExportDestroyInsecureOUT->eError =
		PVRSRVFindHandleUnlocked(psConnection->psProcessHandleBase->psHandleBase,
					&hExportInt,
					psExportInt,
					PVRSRV_HANDLE_TYPE_PVRSRV_FENCE_EXPORT);
	if (psSyncFbFenceExportDestroyInsecureOUT->eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,
		        "PVRSRVBridgeSyncFbFenceExportDestroyInsecure: %s",
		        PVRSRVGetErrorStringKM(psSyncFbFenceExportDestroyInsecureOUT->eError)));
	}
	PVR_ASSERT(psSyncFbFenceExportDestroyInsecureOUT->eError == PVRSRV_OK);

	psSyncFbFenceExportDestroyInsecureOUT->eError =
		PVRSRVReleaseHandleUnlocked(psConnection->psProcessHandleBase->psHandleBase,
					hExportInt,
					PVRSRV_HANDLE_TYPE_PVRSRV_FENCE_EXPORT);
	if ((psSyncFbFenceExportDestroyInsecureOUT->eError != PVRSRV_OK) &&
	    (psSyncFbFenceExportDestroyInsecureOUT->eError != PVRSRV_ERROR_RETRY))
	{
		PVR_DPF((PVR_DBG_ERROR,
		        "PVRSRVBridgeSyncFbFenceExportDestroyInsecure: %s",
		        PVRSRVGetErrorStringKM(psSyncFbFenceExportDestroyInsecureOUT->eError)));
	}
	PVR_ASSERT((psSyncFbFenceExportDestroyInsecureOUT->eError == PVRSRV_OK) ||
	           (psSyncFbFenceExportDestroyInsecureOUT->eError == PVRSRV_ERROR_RETRY));





	psSyncFbFenceExportDestroyInsecureOUT->eError =
		PVRSRVReleaseHandleUnlocked(KERNEL_HANDLE_BASE,
					(IMG_HANDLE) psSyncFbFenceExportDestroyInsecureIN->hExport,
					PVRSRV_HANDLE_TYPE_PVRSRV_FENCE_EXPORT);
	if ((psSyncFbFenceExportDestroyInsecureOUT->eError != PVRSRV_OK) &&
	    (psSyncFbFenceExportDestroyInsecureOUT->eError != PVRSRV_ERROR_RETRY))
	{
		PVR_DPF((PVR_DBG_ERROR,
		        "PVRSRVBridgeSyncFbFenceExportDestroyInsecure: %s",
		        PVRSRVGetErrorStringKM(psSyncFbFenceExportDestroyInsecureOUT->eError)));
		UnlockHandle();
		goto SyncFbFenceExportDestroyInsecure_exit;
	}

	/* Release now we have destroyed handles. */
	UnlockHandle();



SyncFbFenceExportDestroyInsecure_exit:




	return 0;
}

#else
#define PVRSRVBridgeSyncFbFenceExportDestroyInsecure NULL
#endif

#if defined(SUPPORT_INSECURE_EXPORT)
static IMG_INT
PVRSRVBridgeSyncFbFenceImportInsecure(IMG_UINT32 ui32DispatchTableEntry,
					  PVRSRV_BRIDGE_IN_SYNCFBFENCEIMPORTINSECURE *psSyncFbFenceImportInsecureIN,
					  PVRSRV_BRIDGE_OUT_SYNCFBFENCEIMPORTINSECURE *psSyncFbFenceImportInsecureOUT,
					 CONNECTION_DATA *psConnection)
{
	IMG_HANDLE hImport = psSyncFbFenceImportInsecureIN->hImport;
	PVRSRV_FENCE_EXPORT * psImportInt = NULL;
	PVRSRV_FENCE_SERVER * psSyncHandleInt = NULL;







	/* Lock over handle lookup. */
	LockHandle();





					/* Look up the address from the handle */
					psSyncFbFenceImportInsecureOUT->eError =
						PVRSRVLookupHandleUnlocked(KERNEL_HANDLE_BASE,
											(void **) &psImportInt,
											hImport,
											PVRSRV_HANDLE_TYPE_PVRSRV_FENCE_EXPORT,
											IMG_TRUE);
					if(psSyncFbFenceImportInsecureOUT->eError != PVRSRV_OK)
					{
						UnlockHandle();
						goto SyncFbFenceImportInsecure_exit;
					}
	/* Release now we have looked up handles. */
	UnlockHandle();

	psSyncFbFenceImportInsecureOUT->eError =
		SyncFbFenceImportInsecure(psConnection, OSGetDevData(psConnection),
					psImportInt,
					&psSyncHandleInt);
	/* Exit early if bridged call fails */
	if(psSyncFbFenceImportInsecureOUT->eError != PVRSRV_OK)
	{
		goto SyncFbFenceImportInsecure_exit;
	}

	/* Lock over handle creation. */
	LockHandle();





	psSyncFbFenceImportInsecureOUT->eError = PVRSRVAllocHandleUnlocked(psConnection->psProcessHandleBase->psHandleBase,

							&psSyncFbFenceImportInsecureOUT->hSyncHandle,
							(void *) psSyncHandleInt,
							PVRSRV_HANDLE_TYPE_PVRSRV_FENCE_SERVER,
							PVRSRV_HANDLE_ALLOC_FLAG_MULTI
							,(PFN_HANDLE_RELEASE)&SyncFbFenceRelease);
	if (psSyncFbFenceImportInsecureOUT->eError != PVRSRV_OK)
	{
		UnlockHandle();
		goto SyncFbFenceImportInsecure_exit;
	}

	/* Release now we have created handles. */
	UnlockHandle();



SyncFbFenceImportInsecure_exit:

	/* Lock over handle lookup cleanup. */
	LockHandle();







					/* Unreference the previously looked up handle */
					if(psImportInt)
					{
						PVRSRVReleaseHandleUnlocked(KERNEL_HANDLE_BASE,
										hImport,
										PVRSRV_HANDLE_TYPE_PVRSRV_FENCE_EXPORT);
					}
	/* Release now we have cleaned up look up handles. */
	UnlockHandle();

	if (psSyncFbFenceImportInsecureOUT->eError != PVRSRV_OK)
	{
		if (psSyncHandleInt)
		{
			SyncFbFenceRelease(psSyncHandleInt);
		}
	}


	return 0;
}

#else
#define PVRSRVBridgeSyncFbFenceImportInsecure NULL
#endif

static IMG_INT
PVRSRVBridgeSyncFbFenceDump2(IMG_UINT32 ui32DispatchTableEntry,
					  PVRSRV_BRIDGE_IN_SYNCFBFENCEDUMP2 *psSyncFbFenceDump2IN,
					  PVRSRV_BRIDGE_OUT_SYNCFBFENCEDUMP2 *psSyncFbFenceDump2OUT,
					 CONNECTION_DATA *psConnection)
{
	IMG_HANDLE hFence = psSyncFbFenceDump2IN->hFence;
	PVRSRV_FENCE_SERVER * psFenceInt = NULL;
	IMG_CHAR *uiFileStrInt = NULL;
	IMG_CHAR *uiModuleStrInt = NULL;
	IMG_CHAR *uiDescStrInt = NULL;

	IMG_UINT32 ui32NextOffset = 0;
	IMG_BYTE   *pArrayArgsBuffer = NULL;
#if !defined(INTEGRITY_OS)
	IMG_BOOL bHaveEnoughSpace = IMG_FALSE;
#endif

	IMG_UINT32 ui32BufferSize = 
			(psSyncFbFenceDump2IN->ui32FileStrLength * sizeof(IMG_CHAR)) +
			(psSyncFbFenceDump2IN->ui32ModuleStrLength * sizeof(IMG_CHAR)) +
			(psSyncFbFenceDump2IN->ui32DescStrLength * sizeof(IMG_CHAR)) +
			0;





	if (ui32BufferSize != 0)
	{
#if !defined(INTEGRITY_OS)
		/* Try to use remainder of input buffer for copies if possible, word-aligned for safety. */
		IMG_UINT32 ui32InBufferOffset = PVR_ALIGN(sizeof(*psSyncFbFenceDump2IN), sizeof(unsigned long));
		IMG_UINT32 ui32InBufferExcessSize = ui32InBufferOffset >= PVRSRV_MAX_BRIDGE_IN_SIZE ? 0 :
			PVRSRV_MAX_BRIDGE_IN_SIZE - ui32InBufferOffset;

		bHaveEnoughSpace = ui32BufferSize <= ui32InBufferExcessSize;
		if (bHaveEnoughSpace)
		{
			IMG_BYTE *pInputBuffer = (IMG_BYTE *)psSyncFbFenceDump2IN;

			pArrayArgsBuffer = &pInputBuffer[ui32InBufferOffset];
		}
		else
#endif
		{
			pArrayArgsBuffer = OSAllocMemNoStats(ui32BufferSize);

			if(!pArrayArgsBuffer)
			{
				psSyncFbFenceDump2OUT->eError = PVRSRV_ERROR_OUT_OF_MEMORY;
				goto SyncFbFenceDump2_exit;
			}
		}
	}

	if (psSyncFbFenceDump2IN->ui32FileStrLength != 0)
	{
		uiFileStrInt = (IMG_CHAR*)(((IMG_UINT8 *)pArrayArgsBuffer) + ui32NextOffset);
		ui32NextOffset += psSyncFbFenceDump2IN->ui32FileStrLength * sizeof(IMG_CHAR);
	}

			/* Copy the data over */
			if (psSyncFbFenceDump2IN->ui32FileStrLength * sizeof(IMG_CHAR) > 0)
			{
				if ( OSCopyFromUser(NULL, uiFileStrInt, (const void __user *) psSyncFbFenceDump2IN->puiFileStr, psSyncFbFenceDump2IN->ui32FileStrLength * sizeof(IMG_CHAR)) != PVRSRV_OK )
				{
					psSyncFbFenceDump2OUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

					goto SyncFbFenceDump2_exit;
				}
			}
	if (psSyncFbFenceDump2IN->ui32ModuleStrLength != 0)
	{
		uiModuleStrInt = (IMG_CHAR*)(((IMG_UINT8 *)pArrayArgsBuffer) + ui32NextOffset);
		ui32NextOffset += psSyncFbFenceDump2IN->ui32ModuleStrLength * sizeof(IMG_CHAR);
	}

			/* Copy the data over */
			if (psSyncFbFenceDump2IN->ui32ModuleStrLength * sizeof(IMG_CHAR) > 0)
			{
				if ( OSCopyFromUser(NULL, uiModuleStrInt, (const void __user *) psSyncFbFenceDump2IN->puiModuleStr, psSyncFbFenceDump2IN->ui32ModuleStrLength * sizeof(IMG_CHAR)) != PVRSRV_OK )
				{
					psSyncFbFenceDump2OUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

					goto SyncFbFenceDump2_exit;
				}
			}
	if (psSyncFbFenceDump2IN->ui32DescStrLength != 0)
	{
		uiDescStrInt = (IMG_CHAR*)(((IMG_UINT8 *)pArrayArgsBuffer) + ui32NextOffset);
		ui32NextOffset += psSyncFbFenceDump2IN->ui32DescStrLength * sizeof(IMG_CHAR);
	}

			/* Copy the data over */
			if (psSyncFbFenceDump2IN->ui32DescStrLength * sizeof(IMG_CHAR) > 0)
			{
				if ( OSCopyFromUser(NULL, uiDescStrInt, (const void __user *) psSyncFbFenceDump2IN->puiDescStr, psSyncFbFenceDump2IN->ui32DescStrLength * sizeof(IMG_CHAR)) != PVRSRV_OK )
				{
					psSyncFbFenceDump2OUT->eError = PVRSRV_ERROR_INVALID_PARAMS;

					goto SyncFbFenceDump2_exit;
				}
			}

	/* Lock over handle lookup. */
	LockHandle();





					/* Look up the address from the handle */
					psSyncFbFenceDump2OUT->eError =
						PVRSRVLookupHandleUnlocked(psConnection->psProcessHandleBase->psHandleBase,
											(void **) &psFenceInt,
											hFence,
											PVRSRV_HANDLE_TYPE_PVRSRV_FENCE_SERVER,
											IMG_TRUE);
					if(psSyncFbFenceDump2OUT->eError != PVRSRV_OK)
					{
						UnlockHandle();
						goto SyncFbFenceDump2_exit;
					}
	/* Release now we have looked up handles. */
	UnlockHandle();

	psSyncFbFenceDump2OUT->eError =
		SyncFbFenceDump2(
					psFenceInt,
					psSyncFbFenceDump2IN->ui32Line,
					psSyncFbFenceDump2IN->ui32FileStrLength,
					uiFileStrInt,
					psSyncFbFenceDump2IN->ui32ModuleStrLength,
					uiModuleStrInt,
					psSyncFbFenceDump2IN->ui32DescStrLength,
					uiDescStrInt);




SyncFbFenceDump2_exit:

	/* Lock over handle lookup cleanup. */
	LockHandle();







					/* Unreference the previously looked up handle */
					if(psFenceInt)
					{
						PVRSRVReleaseHandleUnlocked(psConnection->psProcessHandleBase->psHandleBase,
										hFence,
										PVRSRV_HANDLE_TYPE_PVRSRV_FENCE_SERVER);
					}
	/* Release now we have cleaned up look up handles. */
	UnlockHandle();

	/* Allocated space should be equal to the last updated offset */
	PVR_ASSERT(ui32BufferSize == ui32NextOffset);

#if defined(INTEGRITY_OS)
	if(pArrayArgsBuffer)
#else
	if(!bHaveEnoughSpace && pArrayArgsBuffer)
#endif
		OSFreeMemNoStats(pArrayArgsBuffer);


	return 0;
}


static IMG_INT
PVRSRVBridgeSyncFbFenceExportSecure(IMG_UINT32 ui32DispatchTableEntry,
					  PVRSRV_BRIDGE_IN_SYNCFBFENCEEXPORTSECURE *psSyncFbFenceExportSecureIN,
					  PVRSRV_BRIDGE_OUT_SYNCFBFENCEEXPORTSECURE *psSyncFbFenceExportSecureOUT,
					 CONNECTION_DATA *psConnection)
{
	IMG_HANDLE hFence = psSyncFbFenceExportSecureIN->hFence;
	PVRSRV_FENCE_SERVER * psFenceInt = NULL;
	PVRSRV_FENCE_EXPORT * psExportInt = NULL;
	CONNECTION_DATA *psSecureConnection;







	/* Lock over handle lookup. */
	LockHandle();





					/* Look up the address from the handle */
					psSyncFbFenceExportSecureOUT->eError =
						PVRSRVLookupHandleUnlocked(psConnection->psProcessHandleBase->psHandleBase,
											(void **) &psFenceInt,
											hFence,
											PVRSRV_HANDLE_TYPE_PVRSRV_FENCE_SERVER,
											IMG_TRUE);
					if(psSyncFbFenceExportSecureOUT->eError != PVRSRV_OK)
					{
						UnlockHandle();
						goto SyncFbFenceExportSecure_exit;
					}
	/* Release now we have looked up handles. */
	UnlockHandle();

	psSyncFbFenceExportSecureOUT->eError =
		SyncFbFenceExportSecure(psConnection, OSGetDevData(psConnection),
					psFenceInt,
					&psSyncFbFenceExportSecureOUT->Export,
					&psExportInt, &psSecureConnection);
	/* Exit early if bridged call fails */
	if(psSyncFbFenceExportSecureOUT->eError != PVRSRV_OK)
	{
		goto SyncFbFenceExportSecure_exit;
	}










SyncFbFenceExportSecure_exit:

	/* Lock over handle lookup cleanup. */
	LockHandle();







					/* Unreference the previously looked up handle */
					if(psFenceInt)
					{
						PVRSRVReleaseHandleUnlocked(psConnection->psProcessHandleBase->psHandleBase,
										hFence,
										PVRSRV_HANDLE_TYPE_PVRSRV_FENCE_SERVER);
					}
	/* Release now we have cleaned up look up handles. */
	UnlockHandle();

	if (psSyncFbFenceExportSecureOUT->eError != PVRSRV_OK)
	{
		if (psExportInt)
		{
			SyncFbFenceExportDestroySecure(psExportInt);
		}
	}


	return 0;
}


static IMG_INT
PVRSRVBridgeSyncFbFenceExportDestroySecure(IMG_UINT32 ui32DispatchTableEntry,
					  PVRSRV_BRIDGE_IN_SYNCFBFENCEEXPORTDESTROYSECURE *psSyncFbFenceExportDestroySecureIN,
					  PVRSRV_BRIDGE_OUT_SYNCFBFENCEEXPORTDESTROYSECURE *psSyncFbFenceExportDestroySecureOUT,
					 CONNECTION_DATA *psConnection)
{









	/* Lock over handle destruction. */
	LockHandle();





	psSyncFbFenceExportDestroySecureOUT->eError =
		PVRSRVReleaseHandleUnlocked(psConnection->psHandleBase,
					(IMG_HANDLE) psSyncFbFenceExportDestroySecureIN->hExport,
					PVRSRV_HANDLE_TYPE_PVRSRV_FENCE_EXPORT);
	if ((psSyncFbFenceExportDestroySecureOUT->eError != PVRSRV_OK) &&
	    (psSyncFbFenceExportDestroySecureOUT->eError != PVRSRV_ERROR_RETRY))
	{
		PVR_DPF((PVR_DBG_ERROR,
		        "PVRSRVBridgeSyncFbFenceExportDestroySecure: %s",
		        PVRSRVGetErrorStringKM(psSyncFbFenceExportDestroySecureOUT->eError)));
		UnlockHandle();
		goto SyncFbFenceExportDestroySecure_exit;
	}

	/* Release now we have destroyed handles. */
	UnlockHandle();



SyncFbFenceExportDestroySecure_exit:




	return 0;
}


static IMG_INT
PVRSRVBridgeSyncFbFenceImportSecure(IMG_UINT32 ui32DispatchTableEntry,
					  PVRSRV_BRIDGE_IN_SYNCFBFENCEIMPORTSECURE *psSyncFbFenceImportSecureIN,
					  PVRSRV_BRIDGE_OUT_SYNCFBFENCEIMPORTSECURE *psSyncFbFenceImportSecureOUT,
					 CONNECTION_DATA *psConnection)
{
	PVRSRV_FENCE_SERVER * psSyncHandleInt = NULL;








	psSyncFbFenceImportSecureOUT->eError =
		SyncFbFenceImportSecure(psConnection, OSGetDevData(psConnection),
					psSyncFbFenceImportSecureIN->Import,
					&psSyncHandleInt);
	/* Exit early if bridged call fails */
	if(psSyncFbFenceImportSecureOUT->eError != PVRSRV_OK)
	{
		goto SyncFbFenceImportSecure_exit;
	}

	/* Lock over handle creation. */
	LockHandle();





	psSyncFbFenceImportSecureOUT->eError = PVRSRVAllocHandleUnlocked(psConnection->psProcessHandleBase->psHandleBase,

							&psSyncFbFenceImportSecureOUT->hSyncHandle,
							(void *) psSyncHandleInt,
							PVRSRV_HANDLE_TYPE_PVRSRV_FENCE_SERVER,
							PVRSRV_HANDLE_ALLOC_FLAG_MULTI
							,(PFN_HANDLE_RELEASE)&SyncFbFenceRelease);
	if (psSyncFbFenceImportSecureOUT->eError != PVRSRV_OK)
	{
		UnlockHandle();
		goto SyncFbFenceImportSecure_exit;
	}

	/* Release now we have created handles. */
	UnlockHandle();



SyncFbFenceImportSecure_exit:



	if (psSyncFbFenceImportSecureOUT->eError != PVRSRV_OK)
	{
		if (psSyncHandleInt)
		{
			SyncFbFenceRelease(psSyncHandleInt);
		}
	}


	return 0;
}




/* *************************************************************************** 
 * Server bridge dispatch related glue 
 */

static IMG_BOOL bUseLock = IMG_FALSE;

PVRSRV_ERROR InitSYNCFALLBACKBridge(void);
PVRSRV_ERROR DeinitSYNCFALLBACKBridge(void);

/*
 * Register all SYNCFALLBACK functions with services
 */
PVRSRV_ERROR InitSYNCFALLBACKBridge(void)
{

	SetDispatchTableEntry(PVRSRV_BRIDGE_SYNCFALLBACK, PVRSRV_BRIDGE_SYNCFALLBACK_SYNCFBTIMELINECREATEPVR, PVRSRVBridgeSyncFbTimelineCreatePVR,
					NULL, bUseLock);

	SetDispatchTableEntry(PVRSRV_BRIDGE_SYNCFALLBACK, PVRSRV_BRIDGE_SYNCFALLBACK_SYNCFBTIMELINERELEASE, PVRSRVBridgeSyncFbTimelineRelease,
					NULL, bUseLock);

	SetDispatchTableEntry(PVRSRV_BRIDGE_SYNCFALLBACK, PVRSRV_BRIDGE_SYNCFALLBACK_SYNCFBFENCEDUP, PVRSRVBridgeSyncFbFenceDup,
					NULL, bUseLock);

	SetDispatchTableEntry(PVRSRV_BRIDGE_SYNCFALLBACK, PVRSRV_BRIDGE_SYNCFALLBACK_SYNCFBFENCEMERGE, PVRSRVBridgeSyncFbFenceMerge,
					NULL, bUseLock);

	SetDispatchTableEntry(PVRSRV_BRIDGE_SYNCFALLBACK, PVRSRV_BRIDGE_SYNCFALLBACK_SYNCFBFENCERELEASE, PVRSRVBridgeSyncFbFenceRelease,
					NULL, bUseLock);

	SetDispatchTableEntry(PVRSRV_BRIDGE_SYNCFALLBACK, PVRSRV_BRIDGE_SYNCFALLBACK_SYNCFBFENCEWAIT, PVRSRVBridgeSyncFbFenceWait,
					NULL, bUseLock);

	SetDispatchTableEntry(PVRSRV_BRIDGE_SYNCFALLBACK, PVRSRV_BRIDGE_SYNCFALLBACK_SYNCFBFENCEDUMP, PVRSRVBridgeSyncFbFenceDump,
					NULL, bUseLock);

	SetDispatchTableEntry(PVRSRV_BRIDGE_SYNCFALLBACK, PVRSRV_BRIDGE_SYNCFALLBACK_SYNCFBTIMELINECREATESW, PVRSRVBridgeSyncFbTimelineCreateSW,
					NULL, bUseLock);

	SetDispatchTableEntry(PVRSRV_BRIDGE_SYNCFALLBACK, PVRSRV_BRIDGE_SYNCFALLBACK_SYNCFBFENCECREATESW, PVRSRVBridgeSyncFbFenceCreateSW,
					NULL, bUseLock);

	SetDispatchTableEntry(PVRSRV_BRIDGE_SYNCFALLBACK, PVRSRV_BRIDGE_SYNCFALLBACK_SYNCFBTIMELINEADVANCESW, PVRSRVBridgeSyncFbTimelineAdvanceSW,
					NULL, bUseLock);

	SetDispatchTableEntry(PVRSRV_BRIDGE_SYNCFALLBACK, PVRSRV_BRIDGE_SYNCFALLBACK_SYNCFBFENCEEXPORTINSECURE, PVRSRVBridgeSyncFbFenceExportInsecure,
					NULL, bUseLock);

	SetDispatchTableEntry(PVRSRV_BRIDGE_SYNCFALLBACK, PVRSRV_BRIDGE_SYNCFALLBACK_SYNCFBFENCEEXPORTDESTROYINSECURE, PVRSRVBridgeSyncFbFenceExportDestroyInsecure,
					NULL, bUseLock);

	SetDispatchTableEntry(PVRSRV_BRIDGE_SYNCFALLBACK, PVRSRV_BRIDGE_SYNCFALLBACK_SYNCFBFENCEIMPORTINSECURE, PVRSRVBridgeSyncFbFenceImportInsecure,
					NULL, bUseLock);

	SetDispatchTableEntry(PVRSRV_BRIDGE_SYNCFALLBACK, PVRSRV_BRIDGE_SYNCFALLBACK_SYNCFBFENCEDUMP2, PVRSRVBridgeSyncFbFenceDump2,
					NULL, bUseLock);

	SetDispatchTableEntry(PVRSRV_BRIDGE_SYNCFALLBACK, PVRSRV_BRIDGE_SYNCFALLBACK_SYNCFBFENCEEXPORTSECURE, PVRSRVBridgeSyncFbFenceExportSecure,
					NULL, bUseLock);

	SetDispatchTableEntry(PVRSRV_BRIDGE_SYNCFALLBACK, PVRSRV_BRIDGE_SYNCFALLBACK_SYNCFBFENCEEXPORTDESTROYSECURE, PVRSRVBridgeSyncFbFenceExportDestroySecure,
					NULL, bUseLock);

	SetDispatchTableEntry(PVRSRV_BRIDGE_SYNCFALLBACK, PVRSRV_BRIDGE_SYNCFALLBACK_SYNCFBFENCEIMPORTSECURE, PVRSRVBridgeSyncFbFenceImportSecure,
					NULL, bUseLock);


	return PVRSRV_OK;
}

/*
 * Unregister all syncfallback functions with services
 */
PVRSRV_ERROR DeinitSYNCFALLBACKBridge(void)
{

	UnsetDispatchTableEntry(PVRSRV_BRIDGE_SYNCFALLBACK, PVRSRV_BRIDGE_SYNCFALLBACK_SYNCFBTIMELINECREATEPVR);

	UnsetDispatchTableEntry(PVRSRV_BRIDGE_SYNCFALLBACK, PVRSRV_BRIDGE_SYNCFALLBACK_SYNCFBTIMELINERELEASE);

	UnsetDispatchTableEntry(PVRSRV_BRIDGE_SYNCFALLBACK, PVRSRV_BRIDGE_SYNCFALLBACK_SYNCFBFENCEDUP);

	UnsetDispatchTableEntry(PVRSRV_BRIDGE_SYNCFALLBACK, PVRSRV_BRIDGE_SYNCFALLBACK_SYNCFBFENCEMERGE);

	UnsetDispatchTableEntry(PVRSRV_BRIDGE_SYNCFALLBACK, PVRSRV_BRIDGE_SYNCFALLBACK_SYNCFBFENCERELEASE);

	UnsetDispatchTableEntry(PVRSRV_BRIDGE_SYNCFALLBACK, PVRSRV_BRIDGE_SYNCFALLBACK_SYNCFBFENCEWAIT);

	UnsetDispatchTableEntry(PVRSRV_BRIDGE_SYNCFALLBACK, PVRSRV_BRIDGE_SYNCFALLBACK_SYNCFBFENCEDUMP);

	UnsetDispatchTableEntry(PVRSRV_BRIDGE_SYNCFALLBACK, PVRSRV_BRIDGE_SYNCFALLBACK_SYNCFBTIMELINECREATESW);

	UnsetDispatchTableEntry(PVRSRV_BRIDGE_SYNCFALLBACK, PVRSRV_BRIDGE_SYNCFALLBACK_SYNCFBFENCECREATESW);

	UnsetDispatchTableEntry(PVRSRV_BRIDGE_SYNCFALLBACK, PVRSRV_BRIDGE_SYNCFALLBACK_SYNCFBTIMELINEADVANCESW);

	UnsetDispatchTableEntry(PVRSRV_BRIDGE_SYNCFALLBACK, PVRSRV_BRIDGE_SYNCFALLBACK_SYNCFBFENCEEXPORTINSECURE);

	UnsetDispatchTableEntry(PVRSRV_BRIDGE_SYNCFALLBACK, PVRSRV_BRIDGE_SYNCFALLBACK_SYNCFBFENCEEXPORTDESTROYINSECURE);

	UnsetDispatchTableEntry(PVRSRV_BRIDGE_SYNCFALLBACK, PVRSRV_BRIDGE_SYNCFALLBACK_SYNCFBFENCEIMPORTINSECURE);

	UnsetDispatchTableEntry(PVRSRV_BRIDGE_SYNCFALLBACK, PVRSRV_BRIDGE_SYNCFALLBACK_SYNCFBFENCEDUMP2);

	UnsetDispatchTableEntry(PVRSRV_BRIDGE_SYNCFALLBACK, PVRSRV_BRIDGE_SYNCFALLBACK_SYNCFBFENCEEXPORTSECURE);

	UnsetDispatchTableEntry(PVRSRV_BRIDGE_SYNCFALLBACK, PVRSRV_BRIDGE_SYNCFALLBACK_SYNCFBFENCEEXPORTDESTROYSECURE);

	UnsetDispatchTableEntry(PVRSRV_BRIDGE_SYNCFALLBACK, PVRSRV_BRIDGE_SYNCFALLBACK_SYNCFBFENCEIMPORTSECURE);



	return PVRSRV_OK;
}
