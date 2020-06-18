# zmq2eti

Can be connected to the [ODR-DabMux](https://github.com/Opendigitalradio/ODR-DabMux) ZMQ endpoint in order to get a cheap ETI output using a Raspberry PI.

***USE AT YOUR OWN RISK!***

It needs to be run as root and on a raspberry 2 or 3 (not tested on 4), that does not use audio output (so no `dtparam=audio=on` in `/boot/config.txt`).

The program will generate three signals on GPIOs:

* HDB+ on GPIO/BCM18 = PIN 12 (PWM0)
* HDB- on GPIO/BCM13 = PIN 33 (PWM1)
* GPIO/BCM27 = PIN 13 changes every 42 ETI Frames between HIGH and LOW to signal activity

In order to create an ETI Signal you need to attach a circuit that creates approximately +2.37V when HDB+ is HIGH and approximately -2.37V when HDB- is HIGH, 0V otherwise.
HDB+ and HDB- will be HIGH for about 244ns.

Sadly I was not able to create the circuit myself, so I cannot share it here. It used one XOR IC and a HCT125 as well as some passive components.
So if you have electrical knowledge and came up with a circuit matching the requirements please feel free to create a pull request.

> The physical parameters of ETI(NI, G.703) shall conform to the requirement of the ITU-T Recommendation G.703 [4] for 2 048 kbit/s interfaces. The minimum requirement shall be a 75 Ω female BNC connector fitted to the equipment.

* [ETI Specification](https://www.etsi.org/deliver/etsi_i_ets/300700_300799/300799/01_30_9733/ets_300799e01v.pdf) (Page 28ff. "Network Independent layer for G.703 interfaces, ETI(NI) or ETI(NI, G.703)")
* [G.703 Specification](https://www.itu.int/rec/T-REC-G.703-201604-I/en) (Page 22ff. "Interface at 2048 kbit/s (E12)")


> THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR ANYONE DISTRIBUTING THE SOFTWARE BE LIABLE FOR ANY DAMAGES OR OTHER LIABILITY, WHETHER IN CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

Inspired by https://github.com/Wallacoloo/Raspberry-Pi-DMA-Example/blob/master/dma-gpio.c
and http://www.icrobotics.co.uk/wiki/index.php/Turning_the_Raspberry_Pi_Into_an_FM_Transmitter
and https://github.com/hzeller/rpi-gpio-dma-demo/blob/master/gpio-dma-test.c
