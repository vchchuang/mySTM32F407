all:
	$(MAKE) -C discoveryF4
	$(MAKE) -C hello-stm32
	$(MAKE) -C neocon

all:
	$(MAKE) -C discoveryF4 clean
	$(MAKE) -C hello-stm32 clean
	$(MAKE) -C neocon clean

