smbios_ss = ss.source_set()
smbios_ss.add(files('smbios.c'))
smbios_ss.add(when: 'CONFIG_IPMI',
              if_true: files('smbios_type_38.c'),
              if_false: files('smbios_type_38-stub.c'))

smbios_ss.add(when: 'CONFIG_RISCV_VIRT',
              if_true: files('smbios_type_44.c'),
              if_false: files('smbios_type_44-stub.c'))

softmmu_ss.add_all(when: 'CONFIG_SMBIOS', if_true: smbios_ss)
softmmu_ss.add(when: 'CONFIG_SMBIOS', if_false: files('smbios-stub.c'))

softmmu_ss.add(when: 'CONFIG_ALL', if_true: files(
  'smbios-stub.c',
  'smbios_type_38-stub.c',
  'smbios_type_44-stub.c',
))
