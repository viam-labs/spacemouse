// Package main runs the spacemouse Viam module.
package main

import (
	"go.viam.com/rdk/components/input"
	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/discovery"

	spacemousedisc "github.com/viam-labs/spacemouse/discovery"
	"github.com/viam-labs/spacemouse/spacemouse"
)

func main() {
	module.ModularMain(
		resource.APIModel{API: input.API, Model: spacemouse.Model},
		resource.APIModel{API: discovery.API, Model: spacemousedisc.Model},
	)
}
