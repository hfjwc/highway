#ifndef GENJAVASO_GLOBAL_H
#define GENJAVASO_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(GENJAVASO_LIBRARY)
#  define GENJAVASO_EXPORT Q_DECL_EXPORT
#else
#  define GENJAVASO_EXPORT Q_DECL_IMPORT
#endif

#endif // GENJAVASO_GLOBAL_H
