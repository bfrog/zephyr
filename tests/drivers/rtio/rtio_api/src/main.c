/*
 * Copyright (c) 2019 Thomas Burdick <thomas.burdick@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @addtogroup t_driver_rtio
 * @{
 * @defgroup t_rtio_basic_api test_rtio_basic_api
 * @}
 */

#include <ztest.h>

extern void test_rtio_mempool_allocator(void);
extern void test_rtio_source_sink(void);

void test_main(void)
{
	ztest_test_suite(rtio_basic_api_test,
			 ztest_unit_test(test_rtio_mempool_allocator),
			 ztest_unit_test(test_rtio_source_sink)
			 );
	ztest_run_test_suite(rtio_basic_api_test);
}
