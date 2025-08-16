// SPDX-License-Identifier: GPL-2.0
// ALSA driver for Marian MARC 2 PCI
//
// This driver is a modernization of the original marc2-linux-0.1.1 driver
// by Frank Lefebvre, adapted for modern (6.x) Linux kernels.
// Reverse engineering and debugging by "Wasz Informatyk" and Gemini AI.
//
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/control.h>
#include <linux/firmware.h>
#include <asm/io.h>

MODULE_AUTHOR("Frank Lefebvre, Wasz Informatyk, Gemini AI");
MODULE_DESCRIPTION("ALSA driver for Marian MARC 2");
MODULE_LICENSE("GPL");
MODULE_FIRMWARE("marc2.bit");

// --- Hardware Definitions from original marc2.c ---
#define REG_CTL      0x00 // Control and Status Register
#define REG_DATA     0x04 // Data transfer Register
#define REG_ADDR     0x08 // Address Register
#define REG_FREQ     0x0C // Samplerate/Clock Register

// Control and Status bits
#define CTL_PLAY_START      (1 << 0)
#define CTL_REC_START       (1 << 1)
#define CTL_IRQ_ENABLE      (1 << 2)
#define CTL_IRQ_PENDING     (1 << 3)
#define CTL_IRQ_ACK         (1 << 23)

// Address register values
#define ADDR_PLAY_COUNTER   0
#define ADDR_REC_COUNTER    1
#define ADDR_CODEC_CTRL     2

// Codec control values
#define CODEC_WRITE         (1 << 16)
#define CODEC_ADDR_MASK     0x1f
#define CODEC_ADDR_SHIFT    8
#define CODEC_DATA_MASK     0xff

// --- Driver data structure ---
struct marc2_chip {
	struct snd_card *card;
	struct pci_dev *pci;
	int irq;

	void __iomem *port; // Base address of FPGA registers

	struct snd_pcm *pcm;
	struct snd_pcm_substream *playback_substream;
	struct snd_pcm_substream *capture_substream;

	spinlock_t lock;
	u32 control_reg;
};

// --- Low-level hardware access functions ---

// Write to a codec register
static void marc2_codec_write(struct marc2_chip *chip, u8 addr, u8 data)
{
	u32 val = CODEC_WRITE |
		  ((addr & CODEC_ADDR_MASK) << CODEC_ADDR_SHIFT) |
		  (data & CODEC_DATA_MASK);
	
	writel(ADDR_CODEC_CTRL, chip->port + REG_ADDR);
	writel(val, chip->port + REG_DATA);
}

// Read a hardware counter
static unsigned int marc2_read_counter(struct marc2_chip *chip, int which)
{
	unsigned long flags;
	unsigned int count;

	spin_lock_irqsave(&chip->lock, flags);
	writel(which, chip->port + REG_ADDR);
	count = readl(chip->port + REG_DATA);
	spin_unlock_irqrestore(&chip->lock, flags);
	return count;
}

// --- Interrupt Handler ---
static irqreturn_t marc2_interrupt(int irq, void *dev_id)
{
	struct marc2_chip *chip = dev_id;
	u32 status;

	status = readl(chip->port + REG_CTL);
	if (!(status & CTL_IRQ_PENDING))
		return IRQ_NONE;

	// Acknowledge interrupt
	writel(status | CTL_IRQ_ACK, chip->port + REG_CTL);
	writel(status & ~CTL_IRQ_ACK, chip->port + REG_CTL);

	if (chip->playback_substream)
		snd_pcm_period_elapsed(chip->playback_substream);
	if (chip->capture_substream)
		snd_pcm_period_elapsed(chip->capture_substream);

	return IRQ_HANDLED;
}

// --- PCM Callbacks ---

static const struct snd_pcm_hardware marc2_pcm_hw = {
	.info = SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID |
		SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER,
	.formats = SNDRV_PCM_FMTBIT_S16_LE, // This card is 16-bit
	.rates = SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000,
	.rate_min = 32000,
	.rate_max = 48000,
	.channels_min = 2,
	.channels_max = 2,
	.buffer_bytes_max = 128 * 1024,
	.period_bytes_min = 4096,
	.period_bytes_max = 64 * 1024,
	.periods_min = 2,
	.periods_max = 32,
};

static int marc2_pcm_open(struct snd_pcm_substream *substream)
{
	struct marc2_chip *chip = snd_pcm_substream_chip(substream);
	substream->runtime->hw = marc2_pcm_hw;
	
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		chip->playback_substream = substream;
	else
		chip->capture_substream = substream;
		
	return 0;
}

static int marc2_pcm_close(struct snd_pcm_substream *substream)
{
	struct marc2_chip *chip = snd_pcm_substream_chip(substream);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		chip->playback_substream = NULL;
	else
		chip->capture_substream = NULL;
		
	return 0;
}

static int marc2_pcm_hw_params(struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *hw_params)
{
	return snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(hw_params));
}

static int marc2_pcm_hw_free(struct snd_pcm_substream *substream)
{
	return snd_pcm_lib_free_pages(substream);
}

// Set sample rate based on original driver logic
static const struct {
	unsigned int rate;
	u32 bits;
} clock_bits[] = {
	{ 48000, 0x00 },
	{ 44100, 0x01 },
	{ 32000, 0x02 },
};

static int marc2_set_rate(struct marc2_chip *chip, unsigned int rate)
{
	int i;
	u32 bits = 0;

	for (i = 0; i < ARRAY_SIZE(clock_bits); i++) {
		if (rate == clock_bits[i].rate) {
			bits = clock_bits[i].bits;
			writel(bits, chip->port + REG_FREQ);
			dev_info(&chip->pci->dev, "Set sample rate to %u Hz (val=0x%x)\n", rate, bits);
			return 0;
		}
	}
	dev_err(&chip->pci->dev, "Unsupported sample rate %d\n", rate);
	return -EINVAL;
}

static int marc2_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct marc2_chip *chip = snd_pcm_substream_chip(substream);
	return marc2_set_rate(chip, substream->runtime->rate);
}

static int marc2_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct marc2_chip *chip = snd_pcm_substream_chip(substream);
	u32 mask;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		mask = CTL_PLAY_START;
	else
		mask = CTL_REC_START;

	spin_lock(&chip->lock);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		chip->control_reg |= mask;
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		chip->control_reg &= ~mask;
		break;
	default:
		spin_unlock(&chip->lock);
		return -EINVAL;
	}
	writel(chip->control_reg, chip->port + REG_CTL);
	spin_unlock(&chip->lock);
	return 0;
}

static snd_pcm_uframes_t marc2_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct marc2_chip *chip = snd_pcm_substream_chip(substream);
	unsigned int count;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		count = marc2_read_counter(chip, ADDR_PLAY_COUNTER);
	else
		count = marc2_read_counter(chip, ADDR_REC_COUNTER);
	
	// The counter seems to be in bytes
	return bytes_to_frames(substream->runtime, count);
}

static struct snd_pcm_ops marc2_pcm_ops = {
	.open = marc2_pcm_open,
	.close = marc2_pcm_close,
	.hw_params = marc2_pcm_hw_params,
	.hw_free = marc2_pcm_hw_free,
	.prepare = marc2_pcm_prepare,
	.trigger = marc2_pcm_trigger,
	.pointer = marc2_pcm_pointer,
};

static int marc2_pcm_create(struct marc2_chip *chip)
{
	int err;
	err = snd_pcm_new(chip->card, "MARC 2 PCM", 0, 1, 1, &chip->pcm);
	if (err < 0) return err;

	chip->pcm->private_data = chip;
	strcpy(chip->pcm->name, "Marian MARC 2");
	snd_pcm_set_ops(chip->pcm, SNDRV_PCM_STREAM_PLAYBACK, &marc2_pcm_ops);
	snd_pcm_set_ops(chip->pcm, SNDRV_PCM_STREAM_CAPTURE, &marc2_pcm_ops);

	snd_pcm_lib_preallocate_pages_for_all(chip->pcm, SNDRV_DMA_TYPE_DEV_SG,
					      &chip->pci->dev, 64*1024, 128*1024);
	return 0;
}


// --- Main Probe and PCI functions ---

static int marc2_load_firmware(struct marc2_chip *chip)
{
	const struct firmware *fw;
	int err;
	int i = 0, j;
    const u8 *data;

	err = request_firmware(&fw, "marc2.bit", &chip->pci->dev);
	if (err < 0) {
		dev_err(&chip->pci->dev, "Cannot load firmware 'marc2.bit'\n");
		return err;
	}

	// This is BAR4, the firmware load register
	void __iomem *fw_port = pci_iomap(chip->pci, 4, 0);
	if (!fw_port) {
		release_firmware(fw);
		return -ENOMEM;
	}
	
    data = fw->data;
    while (i < fw->size && data[i] != 0xff) i++;
    i++;
    msleep(10);
    for (; i < fw->size; i++) {
        for (j = 0; j < 8; j++) {
            writeb((data[i] >> (7 - j)) & 0x01, fw_port);
        }
    }
	pci_iounmap(chip->pci, fw_port);
	release_firmware(fw);
	dev_info(&chip->pci->dev, "Firmware loaded successfully.\n");
	return 0;
}

static void marc2_chip_init(struct marc2_chip *chip)
{
	// Reset FPGA and enable IRQ
	chip->control_reg = CTL_IRQ_ENABLE;
	writel(chip->control_reg, chip->port + REG_CTL);

	// Initialize codecs (un-mute, set volume, etc.)
	// This sequence is based on observation and may need tweaking
	marc2_codec_write(chip, 0x06, 0x7F); // L Vol
	marc2_codec_write(chip, 0x07, 0x7F); // R Vol
	marc2_codec_write(chip, 0x01, 0x03); // Power up
}

static void marc2_free(struct marc2_chip *chip)
{
	if (chip->irq >= 0)
		free_irq(chip->irq, chip);
	if (chip->port)
		pci_iounmap(chip->pci, chip->port);
	pci_release_regions(chip->pci);
	pci_disable_device(chip->pci);
	kfree(chip);
}

static int marc2_probe(struct pci_dev *pci, const struct pci_device_id *id)
{
	struct snd_card *card;
	struct marc2_chip *chip;
	int err;

	err = snd_card_new(&pci->dev, -1, "MARC2", THIS_MODULE, sizeof(*chip), &card);
	if (err < 0) return err;

	chip = card->private_data;
	chip->card = card;
	chip->pci = pci;
	chip->irq = -1;
	spin_lock_init(&chip->lock);

	err = pci_enable_device(pci);
	if (err < 0) goto error;
	
	err = pci_request_regions(pci, "Marian MARC 2");
	if (err < 0) {
		pci_disable_device(pci);
		goto error;
	}

	// This is BAR3, the main register space
	chip->port = pci_iomap(pci, 3, 0);
	if (!chip->port) {
		err = -ENOMEM;
		pci_release_regions(pci);
		pci_disable_device(pci);
		goto error;
	}

	err = marc2_load_firmware(chip);
	if (err < 0) goto error_free_pci;

	err = request_irq(pci->irq, marc2_interrupt, IRQF_SHARED, "snd_marc2", chip);
	if (err < 0) {
		dev_err(&pci->dev, "Cannot grab IRQ %d\n", pci->irq);
		goto error_free_pci;
	}
	chip->irq = pci->irq;

	marc2_chip_init(chip);

	err = marc2_pcm_create(chip);
	if (err < 0) goto error_free_pci;

	strcpy(card->driver, "snd-marc2");
	strcpy(card->shortname, "Marian MARC 2");
	sprintf(card->longname, "Marian MARC 2 at 0x%lx, IRQ %d", (unsigned long)pci_resource_start(pci, 3), chip->irq);

	err = snd_card_register(card);
	if (err < 0) goto error_free_pci;

	pci_set_drvdata(pci, card);
	return 0;

error_free_pci:
	pci_iounmap(chip->pci, chip->port);
	pci_release_regions(pci);
	pci_disable_device(pci);
error:
	snd_card_free(card);
	return err;
}

static void marc2_remove(struct pci_dev *pci)
{
	struct snd_card *card = pci_get_drvdata(pci);
	snd_card_free(card);
}

static const struct pci_device_id marc2_ids[] = {
	{ 0x1382, 0x4008, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, marc2_ids);

static struct pci_driver marc2_driver = {
	.name = "snd-marc2",
	.id_table = marc2_ids,
	.probe = marc2_probe,
	.remove = marc2_remove,
};

module_pci_driver(marc2_driver);
