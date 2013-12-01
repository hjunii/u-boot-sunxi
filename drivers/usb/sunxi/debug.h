/**
 * debug.h
 */

#include "core.h"

#ifdef CONFIG_DEBUG_FS
extern int sunxi_debugfs_init(struct sunxi *);
extern void sunxi_debugfs_exit(struct sunxi *);
#else
static inline int sunxi_debugfs_init(struct sunxi *s)
{  return 0;  }
static inline void sunxi_debugfs_exit(struct sunxi *s)
{  }
#endif
