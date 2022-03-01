SECTIONS {
	.memory : {
		. = 0x000000;
		*(.init);
		. = ALIGN(4);
		*(.text);
		*(*);
		. = ALIGN(4);
		end = .;
	}
}

