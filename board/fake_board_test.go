package board

import (
	"testing"

	"github.com/stretchr/testify/assert"
)

func TestFakeRegistry(t *testing.T) {
	b, err := NewBoard(Config{Model: "fake"})
	assert.Nil(t, err)
	_, ok := b.(*FakeBoard)
	assert.True(t, ok)
}

func TestFakeBoard(t *testing.T) {
	cfg := Config{
		Analogs: []AnalogConfig{{Name: "blue", Pin: "0"}},
		Servos: []ServoConfig{
			{Name: "s1", Pin: "16"},
			{Name: "s2", Pin: "29"},
		},
		DigitalInterrupts: []DigitalInterruptConfig{
			{Name: "i1", Pin: "35"},
			{Name: "i2", Pin: "31", Type: "servo"},
			{Name: "hall-a", Pin: "38"},
			{Name: "hall-b", Pin: "40"},
		},
		Motors: []MotorConfig{
			{
				Name:             "m",
				Pins:             map[string]string{"a": "11", "b": "13", "pwm": "15"},
				Encoder:          "hall-a",
				EncoderB:         "hall-b",
				TicksPerRotation: 100,
			},
		},
	}

	b, err := NewFakeBoard(cfg)
	if err != nil {
		t.Fatal(err)
	}
	defer b.Close()

	assert.Nil(t, b.Servo("s1").Move(15))

	status, err := b.Status()
	if err != nil {
		t.Fatal(err)
	}
	assert.Equal(t, 15, int(status.Servos["s1"].Angle))

}
