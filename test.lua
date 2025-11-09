local led = 0
while true do
	led = led + 1
	if led > 15 then
		led = 0
	end
	enableLEDs(led)
	sleep(1)
end