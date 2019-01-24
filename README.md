Installing googletest.
https://stackoverflow.com/questions/13513905/how-to-setup-googletest-as-a-shared-library-on-linux
Followed second answer for ubuntu.

g++ -Wall -Werror -g -std=c++11 pool_test.cpp -L/usr/lib/ -lgtest -lgtest_main -pthread && ./a.out
