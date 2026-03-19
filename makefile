SCHEMAS = telemetry.fbs commands.fbs

generate: 
	flatc --go --gen-object-api -o api/go $(SCHEMAS)
	flatcc -a -o firmware/shared/flatcc-generated $(SCHEMAS)

clean: 
	rm -rf api/go/Fripuck2
	rm -rf firmware/shared/flatcc-generated/*.h