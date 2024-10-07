from scipy import signal
import numpy as np
class FTtransfer():
    '''
    该类用于固定时间段内侧线数据信号的时频变换
    '''
    def __init__(self,samp_fre = 20,samp_time = 2.5,trans_w = 12,sensor_num = 6):
        self.samp_fre = samp_fre
        self.samp_time = samp_time
        self.trans_window = trans_w
        self.sensor_num = sensor_num
        self.all_buff = np.zeros([self.sensor_num,int(self.samp_fre*self.samp_time)])

        self.samp_num = 0

        #设置STFT后的时频分辨率,可以尝试一次看看，然后设置起来
        self.t_span,self.f_span = self.get_STFT_size()
        #存储STFT变换后的幅值数组
        self.STFT_amp = np.zeros((self.f_span*self.sensor_num,self.t_span))
    def get_STFT_size(self,):
        test_buff = np.ones([1,int(self.samp_fre*self.samp_time)])
        fre, ts, amp = self.STFT(test_buff, self.samp_fre, self.trans_window)
        t_span = np.size(ts)
        f_span = np.size(fre)
        return t_span, f_span

    def STFT(self, x, fs, n):
        f, t, amp = signal.stft(x, fs, nperseg=n)
        z = np.abs(amp.copy())
        return f, t, z

    def STFT_update(self,x):
        x = np.reshape(x,(-1))
        if np.size(x) != self.sensor_num:
            return 0
        self.all_buff[:,0:-1] = self.all_buff[:,1:]
        self.all_buff[:,-1] = x
        if self.samp_num<self.samp_time*self.samp_fre:
            self.samp_num += 1
            return 0
        else:
            for k in range(self.sensor_num):
                fre, ts, amp = self.STFT(self.all_buff[k,:], self.samp_fre, self.trans_window)
                self.STFT_amp[self.f_span*k:self.f_span*(k+1),:] = amp
            return 1
if __name__ == '__main__':
    ALL_STFT = FTtransfer()
    test_pre = np.ones([1,6])
    for i in range(ALL_STFT.samp_time*ALL_STFT.samp_fre+1):
        test_stft = ALL_STFT.STFT_update(test_pre*i)
        print(test_stft)
    print(ALL_STFT.STFT_amp)

