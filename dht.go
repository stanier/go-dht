package dht

import(
	"bytes"
	"fmt"
	"time"
	"errors"
	"github.com/kidoman/embd"
	"github.com/gavv/monotime"
)

type SensorType int

// Implement Stringer interface.
func (this SensorType) String() string {
	if this == DHT11 {
		return "DHT11"
	} else if this == DHT22 {
		return "DHT22"
	} else if this == AM2302 {
		return "AM2302"
	} else {
		return "!!! unknown !!!"
	}
}

const (
	// Most populare sensor
	DHT11 SensorType = iota + 1
	// More expensive and precise than DHT11
	DHT22
	// Aka DHT22
	AM2302 = DHT22
)

// Keep pulse state with how long it lasted.
type Pulse struct {
	Value    byte
	Duration time.Duration
}

type StampedValue struct {
	Value 	int
	Time	time.Duration
}

// Activate sensor and get back bunch of pulses for further decoding.
func dialDHTxxAndGetResponse(pin int, boostPerfFlag bool) ([]Pulse, error) {
	var boost int = 0
	if boostPerfFlag {
		boost = 1
	}

	// Return array: [pulse, duration, pulse, duration, ...]
	arr, err := dialDHTxxAndRead(int(pin), boost)
	if err != nil {
		return nil, err
	}

	pulses := make([]Pulse, len(arr)/2)

	for i := 0; i < len(arr)/2; i++ {
		var value byte = 0

		if arr[i*2] != 0 {
			value = 1
		}
		pulses[i] = Pulse{Value: value,
			Duration: time.Duration(arr[i*2+1]) * time.Microsecond}
	}
	return pulses, nil
}

// TODO write comment to function
func decodeByte(pulses []Pulse, start int) (byte, error) {
	if len(pulses)-start < 16 {
		return 0, fmt.Errorf("Can't decode byte, since range between "+
			"index and array length is less than 16: %d, %d", start, len(pulses))
	}
	var b int = 0
	for i := 0; i < 8; i++ {
		pulseL := pulses[start+i*2]
		pulseH := pulses[start+i*2+1]
		if pulseL.Value != 0 {
			return 0, fmt.Errorf("Low edge value expected at index %d", start+i*2)
		}
		if pulseH.Value == 0 {
			return 0, fmt.Errorf("High edge value expected at index %d", start+i*2+1)
		}
		const HIGH_DUR_MAX = (70 + (70 + 54)) / 2 * time.Microsecond
		// Calc average value between 24us (bit 0) and 70us (bit 1).
		// Everything that less than this param is bit 0, bigger - bit 1.
		const HIGH_DUR_AVG = (24 + (70-24)/2) * time.Microsecond
		if pulseH.Duration > HIGH_DUR_MAX {
			return 0, fmt.Errorf("High edge value duration %v exceed "+
				"expected maximum amount %v", pulseH.Duration, HIGH_DUR_MAX)
		}
		if pulseH.Duration > HIGH_DUR_AVG {
			//fmt.Printf("bit %d is high\n", 7-i)
			b = b | (1 << uint(7-i))
		}
	}
	return byte(b), nil
}

// Decode bunch of pulse read from DHTxx sensors.
// Use pdf specifications from /docs folder to read 5 bytes and
// convert them to temperature and humidity.
func decodeDHT11Pulses(sensorType SensorType, pulses []Pulse) (temperature float32,
	humidity float32, err error) {
	if len(pulses) == 85 {
		pulses = pulses[3:]
	} else if len(pulses) == 84 {
		pulses = pulses[2:]
	} else if len(pulses) == 83 {
		pulses = pulses[1:]
	} else if len(pulses) != 82 {
		printPulseArrayForDebug(pulses)
		return -1, -1, fmt.Errorf("Can't decode pulse array received from "+
			"DHTxx sensor, since incorrect length: %d", len(pulses))
	}
	pulses = pulses[:80]
	// Decode 1st byte
	b0, err := decodeByte(pulses, 0)
	if err != nil {
		return -1, -1, err
	}
	// Decode 2nd byte
	b1, err := decodeByte(pulses, 16)
	if err != nil {
		return -1, -1, err
	}
	// Decode 3rd byte
	b2, err := decodeByte(pulses, 32)
	if err != nil {
		return -1, -1, err
	}
	// Decode 4th byte
	b3, err := decodeByte(pulses, 48)
	if err != nil {
		return -1, -1, err
	}
	// Decode 5th byte: control sum to verify all data received from sensor
	sum, err := decodeByte(pulses, 64)
	if err != nil {
		return -1, -1, err
	}
	// Produce data integrity check
	if sum != byte(b0+b1+b2+b3) {
		err := fmt.Errorf("Control sum %d doesn't match %d (%d+%d+%d+%d)",
			sum, byte(b0+b1+b2+b3), b0, b1, b2, b3)
		return -1, -1, err
	}
	// Debug output for 5 bytes
	log.Debug("Five bytes from DHTxx: [%d, %d, %d, %d, %d]", b0, b1, b2, b3, sum)
	// Extract temprature and humidity depending on sensor type
	temperature, humidity = 0.0, 0.0
	if sensorType == DHT11 {
		humidity = float32(b0)
		temperature = float32(b2)
	} else if sensorType == DHT22 {
		humidity = (float32(b0)*256 + float32(b1)) / 10.0
		temperature = (float32(b2&0x7F)*256 + float32(b3)) / 10.0
		if b2&0x80 != 0 {
			temperature *= -1.0
		}
	}
	if humidity > 100.0 {
		return -1, -1, fmt.Errorf("Humidity value exceed 100%%: %v", humidity)
	}
	// Success
	return temperature, humidity, nil
}

// Print bunch of pulses for debug purpose.
func printPulseArrayForDebug(pulses []Pulse) {
	var buf bytes.Buffer
	for i, pulse := range pulses {
		buf.WriteString(fmt.Sprintf("pulse %3d: %v, %v\n", i,
			pulse.Value, pulse.Duration))
	}
	log.Debug("Pulse count %d:\n%v", len(pulses), buf.String())
}

// Send activation request to DHTxx sensor via specific pin.
// Then decode pulses sent back with asynchronous
// protocol specific for DHTxx sensors.
//
// Input parameters:
// 1) sensor type: DHT11, DHT22 (aka AM2302);
// 2) pin number from GPIO connector to interract with sensor;
// 3) boost GPIO performance flag should be used for old devices
// such as Raspberry PI 1 (this will require root privileges).
//
// Return:
// 1) temperature in Celsius;
// 2) humidity in percent;
// 3) error if present.
func ReadDHTxx(sensorType SensorType, pin int,
	boostPerfFlag bool) (temperature float32, humidity float32, err error) {
	// Activate sensor and read data to pulses array
	pulses, err := dialDHTxxAndGetResponse(pin, boostPerfFlag)
	if err != nil {
		return -1, -1, err
	}
	// Output debug information
	printPulseArrayForDebug(pulses)
	// Decode pulses
	temp, hum, err := decodeDHT11Pulses(sensorType, pulses)
	if err != nil {
		return -1, -1, err
	}
	return temp, hum, nil
}

// Send activation request to DHTxx sensor via specific pin.
// Then decode pulses sent back with asynchronous
// protocol specific for DHTxx sensors. Retry n times in case of failure.
//
// Input parameters:
// 1) sensor type: DHT11, DHT22 (aka AM2302);
// 2) pin number from gadget GPIO to interract with sensor;
// 3) boost GPIO performance flag should be used for old devices
// such as Raspberry PI 1 (this will require root privileges);
// 4) how many times to retry until success either сounter is zeroed.
//
// Return:
// 1) temperature in Celsius;
// 2) humidity in percent;
// 3) number of extra retries data from sensor;
// 4) error if present.
func ReadDHTxxWithRetry(sensorType SensorType, pin int, boostPerfFlag bool,
	retry int) (temperature float32, humidity float32, retried int, err error) {
	retried = 0
	for {
		temp, hum, err := ReadDHTxx(sensorType, pin, boostPerfFlag)
		if err != nil {
			if retry > 0 {
				log.Warning("%v", err)
				retry--
				retried++
				// Sleep before new attempt
				monoSleep(1500 * time.Millisecond)
				continue
			}
			return -1, -1, retried, err
		}
		return temp, hum, retried, nil
	}
}

/*func gpioReadSeqUntilTimeout(p embd.DigitalPin, timeoutMsec int,
		arr *[]int) error {
	var nextT time.Duration
	var lastT time.Duration

	var nextV int

	maxPulseCount := 16000
	//var values [maxPulseCount * 2]int
	var values = make([]int64, maxPulseCount * 2)

	// Set pin in to receive dial response
	if err := p.SetDirection(embd.In); err != nil {
		//setDefaultPriority()
		return err
	}

	lastV, err := p.Read()
	if err != nil {
		fmt.Println("Failed to read value!")
		return err
	}

	k, i := 0, 0
	values[k*2] = int64(lastV)

	lastT = monotime.Now()

	for {
		// Because declarations
		var err error

		nextV, err = p.Read()
		if err != nil {
			fmt.Println("Failed to read value!")
			return err
		}

		if lastV != nextV {
			nextT = monotime.Now()

			i = 0
			k++

			if (k > maxPulseCount - 1) {
				return errors.New(fmt.Sprintf("Pulse count exceed limit in %d\n",
					maxPulseCount))
			}

			values[k*2] = int64(nextV)
			values[k*2-1] = nextT.Nanoseconds() / int64(1000) - lastT.Nanoseconds() / int64(1000)

			lastV = nextV
			lastT = nextT
		}

		i++

		if i - 1 > 20 {
			nextT = monotime.Now()

			if (nextT.Nanoseconds() / int64(1000) - lastT.Nanoseconds() / int64(1000)) / 1000 > int64(timeoutMsec) {
				values[k*2+1] = int64(timeoutMsec * 1000)
				break
			}
		}
	}

	(*arr) = make([]int, (k+1)*2)

	for i = 0; i <= k; i++ {
		(*arr)[i*2] = int(values[i*2])
		(*arr)[i*2+1] = int(values[i*2+1])
	}

	return nil
}*/

func gpioReadSeqUntilTimeout(p embd.DigitalPin, timeoutMsec int) ([]int, error) {
	/* Assuming each iteration takes two microseconds, it should not take more
	 * 	than 5600 microseconds to gather all 40 bits.  Regardless, we should
	 *	leave room for error and be rounding up to the next power of two for
	 *	memory management purposes (and OCD purposes)
	 */
	var values = make([]StampedValue, 8192)

    if err := embd.InitGPIO(); err != nil {
        fmt.Println(err)
        return nil, err
    }
    defer embd.CloseGPIO()

    /*p, err := embd.NewDigitalPin("5")
    if err != nil {
        fmt.Println(err)
        return nil, err
    }
    defer p.Close()*/

    if err := p.SetDirection(embd.In); err != nil {
        fmt.Println(err)
        return nil, err
    }

    for i, _ := range(values) {
        //fmt.Println(monotime.Now().Nanoseconds() - last.Nanoseconds())

        val, err := p.Read()
		stamp := monotime.Now()
        if err != nil {
            fmt.Println(err)
            return nil, err
        }

		//fmt.Println(val)
        values[i] = StampedValue{val, stamp}
    }

	return sift(values, timeoutMsec)
}

/*func sift(arr []StampedValue) ([]int64, error) {
	// Should store no more than roughly 160 values, power of two rounding again
	var result = make([]int64, 1024)
	var populated = make([]bool, 512)

	j := 0

	for i, element := range(arr) {
		fmt.Println(element.Value)
		if populated[j] == false {
			result[j*2] = int64(element.Value)
			result[j*2+1] = int64(0)

			populated[j] = true

			continue
		}

		if int64(element.Value) == result[j] {
			result[j*2+1] += element.Time.Nanoseconds() / int64(1000) -
				arr[i-1].Time.Nanoseconds() / int64(1000)

			continue
		}

		fmt.Println(result[j*2])
		j++
	}

	return result, nil
}*/

func sift(input []StampedValue, timeoutMsec int) ([]int, error) {
	var nextT time.Duration
	var lastT time.Duration

	var nextV int

	j := 0

	maxPulseCount := 32000
	//var values [maxPulseCount * 2]int
	var values = make([]int64, maxPulseCount * 2)

	lastV := input[j].Value

	k, i := 0, 0
	values[k*2] = int64(lastV)

	lastT = input[j].Time

	for {
		j++

		nextV = input[j].Value

		if lastV != nextV {
			nextT = input[j].Time

			i = 0
			k++

			if (k > maxPulseCount - 1) {
				fmt.Println(k)
				return nil, errors.New(fmt.Sprintf("Pulse count exceed limit in %d\n",
					maxPulseCount))
			}

			values[k*2] = int64(nextV)
			values[k*2-1] = nextT.Nanoseconds() / int64(1000) - lastT.Nanoseconds() / int64(1000)

			lastV = nextV
			lastT = nextT
		}

		i++

		if i - 1 > 20 {
			//nextT = monotime.Now()
			nextT= input[i].Time

			if (nextT.Nanoseconds() / int64(1000) - lastT.Nanoseconds() / int64(1000)) / 1000 > int64(timeoutMsec) {
				values[k*2+1] = int64(timeoutMsec * 1000)
				break
			}
		}
	}

	arr := make([]int, (k+1)*2)

	for i = 0; i <= k; i++ {
		arr[i*2] = int(values[i*2])
		arr[i*2+1] = int(values[i*2+1])
	}

	return arr, nil
}

/* This just blinks an LED.  This is extremely unnecessary in this
 * 	implementation as 1)  this has no correlation to DHTxx functionality, 2) the
 *	embd library has a similar function and 3)  it's dead simple to write, why
 *	would you even need the function already in a library?
 *
 *	Regardless, my ranting takes up about as much disk space as this function,
 *	so it doesn't really hurt to include it in the odd case that someone is
 *  actually using it
 */
func blinkNTimes(pin int, n int) error {
	if err := embd.InitGPIO(); err != nil { return err }
	defer embd.CloseGPIO()

	p, err := embd.NewDigitalPin(pin)
	if err != nil { return err }

	if err := p.SetDirection(embd.Out); err != nil { return err }

	for i := 0; i < n; i++ {
		if err := p.Write(embd.High); err != nil { return err }

		monoSleep(100 * time.Millisecond)

		if err := p.Write(embd.Low); err != nil { return err }

		monoSleep(100 * time.Millisecond)
	}
	// Set pin to high
	if err := p.Write(embd.High); err != nil { return err }

	return nil
}

// TODO:  Convert all referenced C functions and variables
func dialDHTxxAndRead(pin int, boostPerfFlag int) ([]int, error) {
	// Initialize the GPIO interface
	if err := embd.InitGPIO(); err != nil {
		return nil, err
	}
    defer embd.CloseGPIO()

	// Open pin
	p, err := embd.NewDigitalPin(pin)
	if err != nil {
		return nil, err
	}
	defer p.Close()

	// Set pin out for dial pulse
	if err := p.SetDirection(embd.Out); err != nil {
		return nil, err
	}

	// Set pin to high
	if err := p.Write(embd.High); err != nil {
		return nil, err
	}

	// Sleep 500 milliseconds
	monoSleep(500 * time.Millisecond)

	// Set pin to low
	if err := p.Write(embd.Low); err != nil {
		return nil, err
	}

	// Sleep 20 milliseconds according to DHTxx specification
	monoSleep(18 * time.Millisecond)

	// Set pin to low
	if err := p.Write(embd.High); err != nil {
		return nil, err
	}

	monoSleep(40 * time.Microsecond)

	// Read data from sensor
	arr, err := gpioReadSeqUntilTimeout(p, 10);
	if err != nil {
		return nil, err
	}

	return arr, nil
}

func monoSleep(duration time.Duration) {
	d := duration.Nanoseconds()

	start := monotime.Now().Nanoseconds()

	for {
		diff := monotime.Now().Nanoseconds() - start

		if diff >= d {
			//fmt.Printf("DIFF: %s\n", diff - d)
			return
		}
	}
}
