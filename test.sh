g++ -Wall -Werror -g -std=c++11 precomputed.cpp pool.cpp utils.cpp pool_test.cpp -o pool_test -L/usr/lib/ -lgtest -lgtest_main -pthread && ./pool_test
