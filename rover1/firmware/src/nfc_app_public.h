/* 
 * File:   nfc_app_public.h
 * Author: MLTop
 *
 * Created on March 28, 2016, 9:10 PM
 */

#ifndef NFC_APP_PUBLIC_H
#define	NFC_APP_PUBLIC_H

#ifdef	__cplusplus
extern "C" {
#endif

BaseType_t addToNFCQ(char msg);
BaseType_t addToNFCQFromISR(char msg);
BaseType_t addToNFCrxQ(char msg);
BaseType_t addToNFCrxQFromISR(char msg);
BaseType_t readFromNFCtxQ(char* msg);
BaseType_t readFromNFCtxQFromISR(char* msg);

#ifdef	__cplusplus
}
#endif

#endif	/* NFC_APP_PUBLIC_H */

