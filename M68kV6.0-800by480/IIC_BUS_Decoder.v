module IIC_BUS_Decoder (
	input unsigned [31:0] Address,
	input IOSelect,
	input AS_L,
	output reg IICO_Enable
);
always@(*) begin
	// defaults output are inactive, override as required later
	IICO_Enable <= 0 ;
	
	// You can chose any address space you like for your IIC Controller but in the example above, 
	// the address range 00408000 – 0040800F has been chosen to avoid conflict with any other IO
	// devices that are already in the system like the LEDs, Timers, LCD display and Switch inputs, so stick to this address. 
	
	if(AS_L == 1'b0 && IOSelect == 1'b1)begin
			if(Address[31:0] >= 32'h0040_8000 && Address[31:0] <= 32'h0040_800F)begin
				IICO_Enable <= 1'b1;
			end
	end
end
endmodule