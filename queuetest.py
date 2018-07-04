
from multiprocessing import Process, Queue
import os, time, random

class test():
    q = None
    # 写数据进程执行的代码:
    def write(self, q):
        print('Process to write: %s' % os.getpid())
        for value in ['A', 'B', 'C']:
            print('Put %s to queue...' % value)
            q.put(value)
            time.sleep(random.random())

    # 读数据进程执行的代码:
    def read(self, q):
        print('Process to read: %s' % os.getpid())
        while True:
            value = q.get(True)
            print('Get %s from queue.' % value)

    def test(self):
        # 父进程创建Queue，并传给各个子进程：
        self.q = Queue()
        qq = self.q 
        pw = Process(target=self.write, args=(qq,))
        pr = Process(target=self.read, args=(qq,))
        # 启动子进程pw，写入:
        pw.start()
        # 启动子进程pr，读取:
        pr.start()
        # 等待pw结束:
        pw.join()
        # pr进程里是死循环，无法等待其结束，只能强行终止:
        pr.terminate()


if __name__=='__main__':
    k = test()
    k.test()


